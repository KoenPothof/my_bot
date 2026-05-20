#!/usr/bin/env python3
import signal
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from action_msgs.msg import GoalStatus
import math


# Mogelijke staten van de state machine
class State:
    IDLE      = "idle"       # wacht op startsignaal
    PLANNING  = "planning"   # bezig met waypoints opsturen naar Nav2
    DRIVING   = "rijdend"    # robot rijdt de route
    WAITING   = "wachten"    # robot is aangekomen bij een waypoint
    COMPLETED = "voltooid"   # alle waypoints zijn afgerond
    STOPPED   = "gestopt"    # handmatig gestopt door operator
    ERROR     = "fout"       # onherstelbare fout tijdens de route


class PatrolNode(Node):

    def __init__(self):
        super().__init__('patrol_node')

        # Beginstate instellen
        self._state = State.IDLE

        # Nav2 action client aanmaken
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        # Publisher voor de huidige staat zodat andere nodes kunnen luisteren
        self._state_pub = self.create_publisher(String, '/patrol_state', 10)

        # Subscribers voor start en stop signalen
        self.create_subscription(Bool, '/start_patrol', self._on_start, 10)
        self.create_subscription(Bool, '/stop_patrol',  self._on_stop,  10)

        # Alleen echte bestemmingen — Nav2 plant zelf de route ertussen
        self._waypoints = [
            self._make_pose(0.0, 0.0, 0.0),
            self._make_pose(7.70654, -7.23268, 1.60696),  # patrouille punt
            self._make_pose(0.0, 0.0, 0.0),               # terug naar basis
        ]

        # Bijhouden welk waypoint als laatste gezien is in feedback
        self._last_waypoint = -1

        # Actief goal handle opslaan zodat we het kunnen cancellen
        self._goal_handle = None

        # Beginstate publiceren zodat andere nodes direct de state kennen
        self._publish_state()
        self.get_logger().info('PatrolNode klaar — wacht op /start_patrol')


    # State veranderen en publiceren naar /patrol_state
    def _set_state(self, new_state: str):
        old_state   = self._state
        self._state = new_state
        self._publish_state()
        self.get_logger().info(f'[STATE] {old_state} → {new_state}')

    def _publish_state(self):
        msg      = String()
        msg.data = self._state
        self._state_pub.publish(msg)


    # Startsignaal ontvangen via /start_patrol
    def _on_start(self, msg: Bool):
        if not msg.data:
            return

        # Alleen starten vanuit inactieve staten
        valid_start_states = (State.IDLE, State.COMPLETED, State.STOPPED, State.ERROR)
        if self._state not in valid_start_states:
            self.get_logger().warn(
                f'Start genegeerd — robot is momenteel: {self._state}')
            return

        self.get_logger().info('Startsignaal ontvangen — route wordt gestart')
        self._last_waypoint = -1
        self._set_state(State.PLANNING)
        self._send_waypoints()


    # Stopsignaal ontvangen via /stop_patrol
    def _on_stop(self, msg: Bool):
        if msg.data and self._state in (State.DRIVING, State.WAITING):
            self.get_logger().info('Stopsignaal ontvangen — route wordt onderbroken')
            self._set_state(State.STOPPED)
            self._cancel_goal()

    def _cancel_goal(self):
        if self._goal_handle is not None:
            self.get_logger().info('Nav2 goal wordt gecanceld')
            cancel_future = self._goal_handle.cancel_goal_async()
            self._goal_handle = None
            # Wacht max 2 seconden tot Nav2 de cancel bevestigt
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
            self.get_logger().info('Nav2 goal gecanceld')


    # Waypoints opsturen naar Nav2 en wachten op bevestiging
    def _send_waypoints(self):
        self.get_logger().info('Wacht tot Nav2 beschikbaar is...')
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 niet beschikbaar na 5 seconden')
            self._set_state(State.ERROR)
            return

        goal       = FollowWaypoints.Goal()
        goal.poses = self._waypoints

        self.get_logger().info(f'Stuur {len(self._waypoints)} waypoints naar Nav2')
        future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self._on_feedback
        )
        future.add_done_callback(self._on_goal_response)


    # Nav2 reageert op het gestuurde doel
    def _on_goal_response(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Nav2 heeft de route geweigerd')
            self._set_state(State.ERROR)
            return

        self.get_logger().info('Route geaccepteerd — robot rijdt nu')
        self._goal_handle = goal_handle
        self._set_state(State.DRIVING)

        # Wachten op het eindresultaat van de route
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)


    # Tussentijdse update over het huidige waypoint ontvangen
    def _on_feedback(self, feedback_msg):
        if self._state not in (State.DRIVING, State.WAITING):
            return

        current = feedback_msg.feedback.current_waypoint
        total   = len(self._waypoints)

        # Nav2 stuurt feedback continu — alleen reageren als waypoint-index verandert
        if current == self._last_waypoint:
            return

        self._last_waypoint = current
        self._set_state(State.WAITING)
        self.get_logger().info(f'Waypoint {current} van {total} bereikt — verder naar {current + 1}')
        self._set_state(State.DRIVING)


    # Eindresultaat van de route verwerken
    def _on_result(self, future):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Route succesvol afgerond!')
            self._set_state(State.COMPLETED)
        elif self._state == State.STOPPED:
            self.get_logger().info('Route gestopt door operator')
        else:
            self.get_logger().error(f'Route mislukt — Nav2 status: {status}')
            self._set_state(State.ERROR)


    # Waypoint aanmaken op basis van positie en richting
    def _make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose                    = PoseStamped()
        pose.header.frame_id    = 'map'
        pose.header.stamp       = self.get_clock().now().to_msg()
        pose.pose.position.x    = x
        pose.pose.position.y    = y
        pose.pose.position.z    = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        return pose


# Startpunt van de node
def main():
    rclpy.init()
    node = PatrolNode()

    # SIGINT (Ctrl+C) zelf afvangen — cancel eerst, dan pas afsluiten
    def _on_shutdown(signum, frame):
        node.get_logger().info('Afsluiten — wacht op Nav2 cancel...')
        node._cancel_goal()
        rclpy.shutdown()

    signal.signal(signal.SIGINT,  _on_shutdown)
    signal.signal(signal.SIGTERM, _on_shutdown)

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()