#!/usr/bin/env python3
import signal
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from action_msgs.msg import GoalStatus
import math

WACHT_SECONDEN = 180  # 3 minuten wachten bij blokkade


class State:
    IDLE      = "idle"
    DRIVING   = "rijdend"
    WAITING   = "wachten"
    COMPLETED = "voltooid"
    STOPPED   = "gestopt"
    ERROR     = "fout"


class PatrolNode(Node):

    def __init__(self):
        super().__init__('patrol_node')

        self._state = State.IDLE
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._state_pub = self.create_publisher(String, '/patrol_state', 10)

        self.create_subscription(Bool, '/start_patrol', self._on_start, 10)
        self.create_subscription(Bool, '/stop_patrol',  self._on_stop,  10)

        self._waypoints = [
            self._make_pose(-0.5, -0.5, 0.0),   # startpunt
            self._make_pose(1.0,  0.0,  0.0),   # waypoint 1
            self._make_pose(-0.5, -0.5, 0.0),   # terug naar start
        ]

        self._current_index       = 0
        self._last_successful_idx = 0
        self._goal_handle         = None
        self._stopped             = False
        self._wait_timer          = None
        self._trying_alternative  = False

        self._publish_state()
        self.get_logger().info('PatrolNode klaar — wacht op /start_patrol')

    # ── State machine ─────────────────────────────────────────────────────────

    def _set_state(self, new_state: str):
        old_state   = self._state
        self._state = new_state
        self._publish_state()
        self.get_logger().info(f'[STATE] {old_state} → {new_state}')

    def _publish_state(self):
        msg      = String()
        msg.data = self._state
        self._state_pub.publish(msg)

    # ── Start / stop ──────────────────────────────────────────────────────────

    def _on_start(self, msg: Bool):
        if not msg.data:
            return
        valid_start_states = (State.IDLE, State.COMPLETED, State.STOPPED, State.ERROR)
        if self._state not in valid_start_states:
            self.get_logger().warn(f'Start genegeerd — robot is momenteel: {self._state}')
            return
        self.get_logger().info('Startsignaal ontvangen — route wordt gestart')
        self._current_index       = 0
        self._last_successful_idx = 0
        self._stopped             = False
        self._trying_alternative  = False
        self._navigate_to_current()

    def _on_stop(self, msg: Bool):
        if msg.data and self._state not in (State.IDLE, State.STOPPED, State.COMPLETED, State.ERROR):
            self.get_logger().info('Stopsignaal ontvangen — route wordt onderbroken')
            self._stopped = True
            self._cancel_wait_timer()
            self._set_state(State.STOPPED)
            self._cancel_goal()

    def _cancel_goal(self):
        if self._goal_handle is not None:
            cancel_future     = self._goal_handle.cancel_goal_async()
            self._goal_handle = None
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)

    def _cancel_wait_timer(self):
        if self._wait_timer is not None:
            self._wait_timer.cancel()
            self._wait_timer = None

    # ── Navigatie ─────────────────────────────────────────────────────────────

    def _navigate_to_current(self):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 niet beschikbaar na 5 seconden')
            self._set_state(State.ERROR)
            return

        goal      = NavigateToPose.Goal()
        goal.pose = self._waypoints[self._current_index]

        label = 'alternatief waypoint' if self._trying_alternative else 'waypoint'
        self.get_logger().info(
            f'Navigeer naar {label} {self._current_index + 1} van {len(self._waypoints)}')
        self._set_state(State.DRIVING)
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav2 heeft het doel geweigerd')
            self._on_blocked()
            return
        self._goal_handle = goal_handle
        future            = goal_handle.get_result_async()
        future.add_done_callback(self._on_result)

    def _on_result(self, future):
        if self._stopped:
            return

        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Waypoint {self._current_index + 1} bereikt')
            self._last_successful_idx = self._current_index
            self._trying_alternative  = False
            self._set_state(State.WAITING)
            self._current_index += 1

            if self._current_index >= len(self._waypoints):
                self.get_logger().info('Route succesvol afgerond!')
                self._set_state(State.COMPLETED)
            else:
                self._navigate_to_current()
        else:
            self._on_blocked()

    # ── Blokkade afhandeling ──────────────────────────────────────────────────

    def _on_blocked(self):
        if self._trying_alternative:
            # Alternatieve route ook geblokkeerd → terugkeren en fout melden
            self.get_logger().warn(
                'Alternatieve route ook geblokkeerd — terugkeren naar vorige positie')
            self._return_to_last()
        else:
            # Eerste blokkade → 3 minuten wachten zodat doorgang vrij kan komen
            self.get_logger().warn(
                f'Blokkade bij waypoint {self._current_index + 1} — '
                f'wacht {WACHT_SECONDEN // 60} minuten op vrijgave doorgang')
            self._set_state(State.WAITING)
            self._wait_timer = self.create_timer(WACHT_SECONDEN, self._on_wacht_voorbij)

    def _on_wacht_voorbij(self):
        self._cancel_wait_timer()

        if self._stopped:
            return

        # Na 3 minuten → probeer alternatieve route via volgend waypoint
        next_index = self._current_index + 1
        if next_index < len(self._waypoints):
            self.get_logger().info(
                f'Wachttijd voorbij — probeer alternatieve route via waypoint {next_index + 1}')
            self._trying_alternative = True
            self._current_index      = next_index
            self._navigate_to_current()
        else:
            self.get_logger().warn(
                'Geen alternatieve route beschikbaar — terugkeren naar vorige positie')
            self._return_to_last()

    # ── Terugkeren ────────────────────────────────────────────────────────────

    def _return_to_last(self):
        goal      = NavigateToPose.Goal()
        goal.pose = self._waypoints[self._last_successful_idx]

        self.get_logger().info(
            f'Terugkeren naar waypoint {self._last_successful_idx + 1}')
        self._set_state(State.RETURNING)
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._on_return_response)

    def _on_return_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                'FOUT: Kan niet terugkeren — operator ingrijpen vereist')
            self._set_state(State.ERROR)
            return
        self._goal_handle = goal_handle
        future            = goal_handle.get_result_async()
        future.add_done_callback(self._on_return_result)

    def _on_return_result(self, future):
        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                'FOUT: Route mislukt — robot teruggekeerd naar vorige positie — '
                'operator ingrijpen vereist')
        else:
            self.get_logger().error(
                'FOUT: Route mislukt — kon ook niet terugkeren — '
                'operator ingrijpen vereist')
        self._set_state(State.ERROR)

    # ── Hulpfuncties ──────────────────────────────────────────────────────────

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
