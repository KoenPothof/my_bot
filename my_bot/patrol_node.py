#!/usr/bin/env python3
import signal
import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from action_msgs.msg import GoalStatus
import math


FIRST_WAIT_SEC     = 120.0  # 2 minuten wachten voor eerste herpoging — geeft mensen tijd om aan de kant te gaan
RETRY_INTERVAL_SEC = 30.0   # daarna elke 30 seconden opnieuw proberen
BLOCK_TIMEOUT_SEC  = 180.0  # na 3 minuten totaal → volgend waypoint


class State:
    IDLE      = "idle"          # wacht op startsignaal
    PLANNING  = "planning"      # bezig met opsturen naar Nav2
    DRIVING   = "rijdend"       # robot rijdt naar een waypoint
    WAITING   = "wachten"       # korte pauze tussen waypoints
    BLOCKED   = "geblokkeerd"   # weg geblokkeerd, wacht op vrije doorgang
    COMPLETED = "voltooid"      # alle waypoints afgerond
    STOPPED   = "gestopt"       # handmatig gestopt door operator
    ERROR     = "fout"          # onherstelbare fout


class PatrolNode(Node):

    def __init__(self):
        super().__init__('patrol_node')

        self._state = State.IDLE

        # NavigateToPose direct — zelfde als ros2 action send_goal /navigate_to_pose
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self._state_pub = self.create_publisher(String, '/patrol_state', 10)

        self.create_subscription(Bool, '/start_patrol', self._on_start, 10)
        self.create_subscription(Bool, '/stop_patrol',  self._on_stop,  10)

        self._waypoint_coords = [
            (0.0,   0.0,  0.0),
            (0.64,  0.15, 0.0),
            (-0.31, 1.38, 0.0),
            (0.0,   0.0,  0.0),
        ]

        self._current_waypoint = 0
        self._goal_handle      = None
        self._stop_requested   = False
        self._next_timer       = None
        self._retry_timer      = None
        self._block_start_ns   = None   # tijdstip eerste blokkade (nanoseconden)
        self._retreating       = False  # robot rijdt terug naar vorig waypoint
        self._has_retreated    = False  # al één keer teruggereden geweest

        self._publish_state()
        self.get_logger().info('PatrolNode klaar — wacht op /start_patrol')

    def _set_state(self, new_state: str):
        old = self._state
        self._state = new_state
        self._publish_state()
        self.get_logger().info(f'[STATE] {old} → {new_state}')

    def _publish_state(self):
        msg      = String()
        msg.data = self._state
        self._state_pub.publish(msg)

    def _on_start(self, msg: Bool):
        if not msg.data:
            return
        valid = (State.IDLE, State.COMPLETED, State.STOPPED, State.ERROR)
        if self._state not in valid:
            self.get_logger().warn(f'Start genegeerd — robot is momenteel: {self._state}')
            return
        self.get_logger().info('Startsignaal ontvangen — route wordt gestart')
        self._stop_requested   = False
        self._current_waypoint = 0
        self._block_start_ns   = None
        self._retreating       = False
        self._has_retreated    = False
        self._set_state(State.PLANNING)
        self._navigate_to_next()

    def _on_stop(self, msg: Bool):
        if msg.data and self._state in (State.DRIVING, State.WAITING, State.BLOCKED):
            self.get_logger().info('Stopsignaal ontvangen — route wordt onderbroken')
            self._stop_requested = True
            self._cancel_timers()
            self._set_state(State.STOPPED)
            self._cancel_goal()

    def _cancel_goal(self):
        if self._goal_handle is not None:
            self.get_logger().info('Nav2 goal wordt gecanceld')
            cancel_future = self._goal_handle.cancel_goal_async()
            self._goal_handle = None
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)

    def _cancel_timers(self):
        if self._next_timer is not None:
            self._next_timer.cancel()
            self._next_timer = None
        if self._retry_timer is not None:
            self._retry_timer.cancel()
            self._retry_timer = None

    def _navigate_to_next(self):
        if self._stop_requested:
            return

        total = len(self._waypoint_coords)
        if self._current_waypoint >= total:
            self.get_logger().info('Alle waypoints afgerond!')
            self._block_start_ns = None
            self._set_state(State.COMPLETED)
            return

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 niet beschikbaar na 5 seconden')
            self._set_state(State.ERROR)
            return

        x, y, yaw = self._waypoint_coords[self._current_waypoint]

        goal      = NavigateToPose.Goal()
        goal.pose = self._make_pose(x, y, yaw)

        self.get_logger().info(
            f'Navigeer naar waypoint {self._current_waypoint + 1}/{total}: ({x:.2f}, {y:.2f})')
        self._set_state(State.DRIVING)

        future = self._action_client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    def _navigate_to_previous(self):
        if self._stop_requested:
            return

        # Als er geen vorig waypoint is, blijf op huidige positie in BLOCKED state
        if self._current_waypoint == 0:
            self.get_logger().warn('Geen vorig waypoint beschikbaar — wacht op huidige positie')
            self._retreating = False
            self._set_state(State.BLOCKED)
            self._retry_timer = self.create_timer(RETRY_INTERVAL_SEC, self._on_retry_timer)
            return

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 niet beschikbaar na 5 seconden')
            self._set_state(State.ERROR)
            return

        prev_index    = self._current_waypoint - 1
        x, y, yaw     = self._waypoint_coords[prev_index]
        goal          = NavigateToPose.Goal()
        goal.pose     = self._make_pose(x, y, yaw)

        self.get_logger().info(
            f'Rijdt terug naar waypoint {prev_index + 1}: ({x:.2f}, {y:.2f})')

        future = self._action_client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                f'Waypoint {self._current_waypoint + 1} geweigerd door Nav2')
            self._set_state(State.ERROR)
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, _feedback_msg):
        pass

    def _on_result(self, future):
        if self._stop_requested:
            return

        result = future.result()
        status = result.status
        total  = len(self._waypoint_coords)

        if status == GoalStatus.STATUS_SUCCEEDED:
            if self._retreating:
                # Teruggereden naar vorig waypoint — één laatste poging, daarna ERROR
                self._retreating    = False
                self._has_retreated = True
                self.get_logger().info(
                    f'Terug bij waypoint {self._current_waypoint}/{total} — '
                    f'één laatste poging naar waypoint {self._current_waypoint + 1}')
                self._set_state(State.BLOCKED)
                self._retry_timer = self.create_timer(RETRY_INTERVAL_SEC, self._on_retry_timer)
            else:
                self.get_logger().info(
                    f'Waypoint {self._current_waypoint + 1}/{total} bereikt')
                self._block_start_ns = None
                self._has_retreated  = False
                self._set_state(State.WAITING)
                self._current_waypoint += 1
                self._next_timer = self.create_timer(1.0, self._on_next_timer)

        elif status in (GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED):
            self._handle_blockage()

        else:
            self.get_logger().error(
                f'Waypoint {self._current_waypoint + 1} mislukt — status: {status}')
            self._set_state(State.ERROR)

    def _handle_blockage(self):
        if self._has_retreated:
            self.get_logger().error(
                f'Waypoint {self._current_waypoint + 1} nog steeds niet bereikbaar '
                f'na terugrijden — route afgebroken')
            self._set_state(State.ERROR)
            return

        now_ns = self.get_clock().now().nanoseconds

        if self._block_start_ns is None:
            # Eerste blokkade — wacht 2 minuten zodat mensen de tijd hebben om aan de kant te gaan
            self._block_start_ns = now_ns
            self.get_logger().warn(
                f'Weg geblokkeerd bij waypoint {self._current_waypoint + 1} — '
                f'wacht {int(FIRST_WAIT_SEC / 60)} minuten voor eerste herpoging')
            self._set_state(State.BLOCKED)
            self._retry_timer = self.create_timer(FIRST_WAIT_SEC, self._on_retry_timer)
            return

        elapsed = (now_ns - self._block_start_ns) / 1e9

        if elapsed >= BLOCK_TIMEOUT_SEC:
            self.get_logger().warn(
                f'{int(BLOCK_TIMEOUT_SEC / 60)} minuten verstreken — '
                f'rijdt terug naar vorig waypoint {self._current_waypoint}')
            self._block_start_ns = None
            self._retreating     = True
            self._set_state(State.PLANNING)
            self._navigate_to_previous()
        else:
            remaining = int(BLOCK_TIMEOUT_SEC - elapsed)
            self.get_logger().info(
                f'Nog {remaining}s wachten — opnieuw proberen over {int(RETRY_INTERVAL_SEC)}s')
            self._retry_timer = self.create_timer(
                RETRY_INTERVAL_SEC, self._on_retry_timer)

    def _on_next_timer(self):
        self._cancel_timers()
        if not self._stop_requested:
            self._set_state(State.DRIVING)
            self._navigate_to_next()

    def _on_retry_timer(self):
        self._cancel_timers()
        if not self._stop_requested:
            self.get_logger().info(
                f'Opnieuw proberen waypoint {self._current_waypoint + 1}...')
            self._navigate_to_next()

    def _make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose                    = PoseStamped()
        pose.header.frame_id    = 'map'
        pose.header.stamp       = rclpy.time.Time().to_msg()
        pose.pose.position.x    = x
        pose.pose.position.y    = y
        pose.pose.position.z    = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        return pose


def main():
    rclpy.init()
    node = PatrolNode()

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