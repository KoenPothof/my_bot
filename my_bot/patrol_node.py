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

        # Waypoints definiëren — pas coördinaten aan naar jouw kaart
        self._waypoints = [
            self._make_pose(0.0, 0.0,  0.0),        # startpunt / origin
            self._make_pose(3.0, 0.0,  0.0),        # punt 2 → rechts
            self._make_pose(3.0, 3.0,  math.pi/2),  # punt 3 → omhoog
            self._make_pose(0.0, 3.0,  math.pi),    # punt 4 → links
            self._make_pose(0.0, 0.0, -math.pi/2),  # terug naar origin
        ]

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
        self._set_state(State.PLANNING)
        self._send_waypoints()


    # Stopsignaal ontvangen via /stop_patrol
    def _on_stop(self, msg: Bool):
        if msg.data and self._state == State.DRIVING:
            self.get_logger().info('Stopsignaal ontvangen — route wordt onderbroken')
            self._set_state(State.STOPPED)


    # Waypoints opsturen naar Nav2 en wachten op bevestiging
    def _send_waypoints(self):
        self.get_logger().info('Wacht tot Nav2 beschikbaar is...')
        self._action_client.wait_for_server()

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
        self._set_state(State.DRIVING)

        # Wachten op het eindresultaat van de route
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)


    # Tussentijdse update over het huidige waypoint ontvangen
    def _on_feedback(self, feedback_msg):
        current = feedback_msg.feedback.current_waypoint
        total   = len(self._waypoints)

        if self._state == State.DRIVING:
            self._set_state(State.WAITING)
            self.get_logger().info(
                f'Waypoint {current + 1} van {total} bereikt — even wachten')
        elif self._state == State.WAITING:
            self._set_state(State.DRIVING)
            self.get_logger().info(
                f'Verder rijden naar waypoint {current + 1} van {total}')


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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()