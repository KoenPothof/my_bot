#!/usr/bin/env python3
import signal
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
import math

WACHT_SECONDEN  = 120  # 2 minuten wachten bij blokkade
NAV_TIMEOUT_SEC = 60   # seconden per waypoint zonder voortgang → fout
PERSOON_WACHT_SECONDEN = 120
BACKUP_SPEED    = 0.1
BACKUP_DISTANCE = 0.3
BACKUP_CMD_VEL_TOPIC = '/cmd_vel'


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

        # Hoofdnavigatie: NavigateThroughPoses (alle waypoints in één goal)
        self._action_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses')
        # Terugkeer: NavigateToPose (één waypoint)
        self._return_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        self._state_pub     = self.create_publisher(String, '/patrol_state', 10)
        self._buzzer_pub    = self.create_publisher(Bool,   '/buzzer',       10)
        self._indicator_pub = self.create_publisher(String, '/indicators',   10)

        planner_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._planner_pub = self.create_publisher(String, '/planner_selector', planner_qos)

        self.create_subscription(Bool, '/start_patrol', self._on_start, 10)
        self.create_subscription(Bool, '/stop_patrol',  self._on_stop,  10)

        self._select_gridbased_planner()

        self._waypoints = [
            self._make_pose(7.66, -3.49, 0.0),
            self._make_pose(5.59, -3.49, 0.0),
            self._make_pose(6.59, 2.49, 0.0),
            self._make_pose(5.66, 2.036, 0.0),
        ]

        # Cancel services: navigate_through_poses (huidig) + navigate_to_pose (legacy)
        self._cancel_ntps_client = self.create_client(
            CancelGoal, '/navigate_through_poses/_action/cancel_goal')
        self._cancel_ntp_client = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal')

        self._ctrl_lifecycle_client = self.create_client(
            ChangeState, '/controller_server/change_state')
        self._pending_start_timer = None

        self._cmdvel_pub        = self.create_publisher(Twist, BACKUP_CMD_VEL_TOPIC, 10)
        self._safety_wait_timer = None
        self._backup_timer      = None
        self._backup_start      = None
        self._backing_up        = False
        self._blockage_level    = 0
        self._beep_timer        = None

        self._current_index       = 0
        self._route_start_index   = 0  # index waarbij de actieve NavigateThroughPoses begon
        self._last_successful_idx = 0
        self._goal_handle         = None
        self._stopped             = False
        self._wait_timer          = None
        self._trying_alternative  = False
        self._driving_since       = None

        self._safety_stopped    = False
        self._cmdvel_zero_since = None
        self._was_moving        = False

        # Veiligheidsfunctionaliteit tijdelijk uitgeschakeld voor debugging
        # self.create_subscription(Odometry, '/odom', self._on_odom_safety, 10)
        # self.create_timer(0.5, self._check_safety_stop)
        self.create_timer(5.0, self._check_nav_timeout)

        self._publish_state()
        self.get_logger().info('PatrolNode klaar — wacht op /start_patrol')

    def _select_gridbased_planner(self):
        msg = String()
        msg.data = 'GridBased'
        self._planner_pub.publish(msg)

    # ── State machine ─────────────────────────────────────────────────────────

    def _set_state(self, new_state: str):
        old_state            = self._state
        self._state          = new_state
        self._safety_stopped = False
        self._cmdvel_zero_since = None
        self._was_moving     = False
        self._driving_since  = self.get_clock().now() if new_state == State.DRIVING else None
        self._publish_state()
        self.get_logger().info(f'[STATE] {old_state} → {new_state}')

    def _publish_state(self):
        published = (
            State.WAITING
            if self._safety_stopped and self._state == State.DRIVING
            else self._state
        )
        msg = String()
        msg.data = published
        self._state_pub.publish(msg)

    # ── Start / stop ──────────────────────────────────────────────────────────

    def _on_start(self, msg: Bool):
        if not msg.data:
            return
        valid_start_states = (State.IDLE, State.COMPLETED, State.STOPPED, State.ERROR)
        if self._state not in valid_start_states:
            self.get_logger().warn(f'Start genegeerd — robot is momenteel: {self._state}')
            return
        # Altijd vanaf het begin: geen voortgang onthouden tussen sessies.
        self.get_logger().info('Startsignaal ontvangen — route start bij waypoint 1')
        self._select_gridbased_planner()
        self._current_index       = 0
        self._route_start_index   = 0
        self._last_successful_idx = 0
        self._stopped             = False
        self._trying_alternative  = False
        self._blockage_level      = 0
        self._cancel_all_nav2_goals()

    def _cancel_all_nav2_goals(self):
        """Cancel actieve doelen op beide action servers."""
        pending = [0]

        def _on_done(future, name):
            try:
                n = len(future.result().goals_canceling)
                self.get_logger().info(f'{name}: {n} zombie-doel(en) geannuleerd')
            except Exception as exc:
                self.get_logger().warn(f'CancelGoal {name} fout: {exc}')
            pending[0] -= 1
            if pending[0] <= 0:
                self._reset_controller_server()

        req = CancelGoal.Request()

        if self._cancel_ntps_client.service_is_ready():
            pending[0] += 1
            f = self._cancel_ntps_client.call_async(req)
            f.add_done_callback(lambda fut: _on_done(fut, 'NavigateThroughPoses'))

        if self._cancel_ntp_client.service_is_ready():
            pending[0] += 1
            f = self._cancel_ntp_client.call_async(req)
            f.add_done_callback(lambda fut: _on_done(fut, 'NavigateToPose'))

        if pending[0] == 0:
            self.get_logger().warn('Geen cancel-services beschikbaar — sla cancel over')
            self._reset_controller_server()

    def _reset_controller_server(self):
        if not self._ctrl_lifecycle_client.service_is_ready():
            self.get_logger().warn(
                'controller_server lifecycle-service niet bereikbaar — sla reset over')
            self._schedule_start_navigation()
            return
        req = ChangeState.Request()
        req.transition.id = Transition.TRANSITION_DEACTIVATE
        self.get_logger().info('controller_server deactiveren om interne staat te wissen...')
        future = self._ctrl_lifecycle_client.call_async(req)
        future.add_done_callback(self._on_controller_reset_done)

    def _on_controller_reset_done(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info(
                    'controller_server gedeactiveerd — lifecycle_manager herstart hem')
            else:
                self.get_logger().warn('controller_server deactivate mislukt — toch doorgaan')
        except Exception as exc:
            self.get_logger().warn(f'controller_server lifecycle fout: {exc}')
        self._schedule_start_navigation(delay=8.0)

    def _schedule_start_navigation(self, delay: float = 1.0):
        if self._pending_start_timer is not None:
            self._pending_start_timer.cancel()
        self._pending_start_timer = self.create_timer(delay, self._do_start_navigation)

    def _do_start_navigation(self):
        if self._pending_start_timer is not None:
            self._pending_start_timer.cancel()
            self._pending_start_timer = None
        if self._stopped:
            return
        self.get_logger().info('Zombie-doelen verwerkt — route wordt gestart')
        self._navigate_through_remaining()

    def _on_stop(self, msg: Bool):
        if msg.data and self._state not in (State.IDLE, State.STOPPED, State.COMPLETED, State.ERROR):
            self.get_logger().info('Stopsignaal ontvangen — route wordt onderbroken')
            self._stopped = True
            self._cancel_wait_timer()
            self._cancel_safety_wait_timer()
            self._set_state(State.STOPPED)
            self._cancel_goal()

    def _cancel_goal(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    def shutdown_cleanly(self):
        if self._goal_handle is None:
            return
        self.get_logger().info('Afsluiten — annuleer actief Nav2-doel...')
        try:
            future = self._goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        except Exception as exc:
            self.get_logger().warn(f'Kon doel niet netjes annuleren: {exc}')
        finally:
            self._goal_handle = None

    def _cancel_wait_timer(self):
        if self._wait_timer is not None:
            self._wait_timer.cancel()
            self._wait_timer = None

    # ── Navigatie ─────────────────────────────────────────────────────────────

    def _navigate_through_remaining(self):
        """Stuur waypoints vanaf _current_index als één NavigateThroughPoses goal.
        Nav2 navigeert er continu doorheen zonder tussenstops."""
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'Nav2 navigate_through_poses niet beschikbaar na 5 seconden')
            self._set_state(State.ERROR)
            return

        remaining_poses = self._waypoints[self._current_index:]
        now = self.get_clock().now().to_msg()
        for p in remaining_poses:
            p.header.stamp = now

        goal = NavigateThroughPoses.Goal()
        goal.poses = remaining_poses
        self._route_start_index = self._current_index

        label = 'alternatief' if self._trying_alternative else 'route'
        self.get_logger().info(
            f'Stuur {label}: {len(remaining_poses)} waypoints '
            f'(waypoint {self._current_index + 1} t/m {len(self._waypoints)})')
        self._set_state(State.DRIVING)
        self._set_indicator('uit')   # eventuele gevaarslichten uit nu we weer rijden
        future = self._action_client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    def _on_feedback(self, feedback_msg):
        """Reset de per-waypoint timeout telkens als een waypoint gepasseerd wordt."""
        remaining = feedback_msg.feedback.number_of_poses_remaining
        total_sent = len(self._waypoints) - self._route_start_index
        passed = total_sent - remaining
        new_index = self._route_start_index + passed

        if new_index > self._current_index:
            old = self._current_index
            self._current_index = new_index
            self._last_successful_idx = old
            self._driving_since = self.get_clock().now()  # reset timeout per waypoint
            self.get_logger().info(
                f'Waypoint {old + 1} gepasseerd → navigeert naar {self._current_index + 1} '
                f'({remaining} resterend)')

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            retries = getattr(self, '_goal_retries', 0)
            if retries < 1:
                self._goal_retries = retries + 1
                self.get_logger().warn(
                    f'Nav2 heeft het doel geweigerd — retry over 4s (poging {retries + 1}/1)')
                self._schedule_start_navigation(delay=4.0)
            else:
                self._goal_retries = 0
                self.get_logger().error('Nav2 heeft het doel geweigerd (ook na retry)')
                self._on_blocked()
            return
        self._goal_retries = 0
        self._goal_handle = goal_handle
        future = goal_handle.get_result_async()
        future.add_done_callback(self._on_result)

    def _on_result(self, future):
        if self._stopped or self._backing_up:
            return

        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Volledige route succesvol afgerond! Herstart de lus.')
            self._last_successful_idx = len(self._waypoints) - 1
            self._trying_alternative  = False
            self._blockage_level      = 0
            self._set_state(State.WAITING)
            # Loop: begin opnieuw bij waypoint 1
            self._current_index     = 0
            self._route_start_index = 0
            self._schedule_start_navigation(delay=2.0)
        else:
            self.get_logger().warn(
                f'Nav2 route mislukt — status: {status} '
                f'(4=SUCCEEDED, 5=CANCELED, 6=ABORTED)')
            self._on_blocked()

    # ── Blokkade afhandeling ──────────────────────────────────────────────────

    def _on_blocked(self):
        if self._stopped:
            return
        if self._blockage_level == 0:
            self.get_logger().warn(
                f'Blokkade bij waypoint {self._current_index + 1} — '
                f'wacht {WACHT_SECONDEN // 60} min op vrije doorgang')
            self._set_state(State.WAITING)
            self._wait_timer = self.create_timer(WACHT_SECONDEN, self._on_wacht_voorbij)
        else:
            self._escalate_blockage()

    def _on_wacht_voorbij(self):
        self._cancel_wait_timer()
        if self._stopped:
            return
        self._escalate_blockage()

    def _escalate_blockage(self):
        if self._stopped:
            return

        if self._blockage_level == 0:
            self.get_logger().warn('Blokkade — piep, stukje achteruit en opnieuw proberen')
            self._blockage_level = 1
            self._cancel_goal()
            self._beep()
            self._set_indicator('gevaar')   # gevaarslichten aan tijdens blokkade/achteruit
            self._start_backup()

        elif self._blockage_level == 1:
            self._blockage_level = 2
            next_index = self._current_index + 1
            if next_index < len(self._waypoints):
                self.get_logger().warn(
                    f'Nog steeds geblokkeerd — alternatieve route via waypoint {next_index + 1}')
                self._trying_alternative = True
                self._current_index      = next_index
                self._navigate_through_remaining()
            else:
                self.get_logger().warn('Geen alternatief — terugkeren naar vorig waypoint')
                self._return_to_last()
        else:
            self.get_logger().warn('Alternatieve route ook geblokkeerd — terugkeren')
            self._return_to_last()

    def _beep(self):
        on = Bool()
        on.data = True
        self._buzzer_pub.publish(on)

        def _stop_beep():
            off = Bool()
            off.data = False
            self._buzzer_pub.publish(off)
            if self._beep_timer is not None:
                self._beep_timer.cancel()
                self._beep_timer = None

        self._beep_timer = self.create_timer(0.3, _stop_beep)

    def _set_indicator(self, state: str):
        """Stuur de knipperlichten direct aan, net als de buzzer via _beep().
        Het bericht gaat via /indicators → mqtt_hmi_bridge → MQTT → HMI (Q01/Q02).
        state: 'links', 'rechts', 'gevaar' (beide) of 'uit'."""
        msg = String()
        msg.data = state
        self._indicator_pub.publish(msg)

    # ── Terugkeren ────────────────────────────────────────────────────────────

    def _return_to_last(self):
        goal = NavigateToPose.Goal()
        goal.pose = self._waypoints[self._last_successful_idx]
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(
            f'Terugkeren naar waypoint {self._last_successful_idx + 1}')
        self._set_state(State.DRIVING)
        future = self._return_client.send_goal_async(goal)
        future.add_done_callback(self._on_return_response)

    def _on_return_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                'FOUT: Kan niet terugkeren — operator ingrijpen vereist')
            self._set_state(State.ERROR)
            return
        self._goal_handle = goal_handle
        future = goal_handle.get_result_async()
        future.add_done_callback(self._on_return_result)

    def _on_return_result(self, future):
        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                'FOUT: Route mislukt — robot teruggekeerd — operator ingrijpen vereist')
        else:
            self.get_logger().error(
                'FOUT: Route mislukt — kon ook niet terugkeren — operator ingrijpen vereist')
        self._set_state(State.ERROR)

    # ── EZ-Wheel veiligheidsstop monitoring ───────────────────────────────────

    def _on_odom_safety(self, msg: Odometry):
        vx    = msg.twist.twist.linear.x
        vy    = msg.twist.twist.linear.y
        omega = msg.twist.twist.angular.z
        speed = math.sqrt(vx**2 + vy**2)
        moving = speed > 0.01 or abs(omega) > 0.03

        if not moving:
            if self._was_moving and self._cmdvel_zero_since is None:
                self._cmdvel_zero_since = self.get_clock().now()
        else:
            self._was_moving = True
            self._cmdvel_zero_since = None
            if self._safety_stopped:
                self._safety_stopped = False
                self._cancel_safety_wait_timer()
                self._publish_state()
                self.get_logger().info('EZ-Wheel veiligheidsstop opgeheven — robot rijdt verder')

    def _check_safety_stop(self):
        if self._state != State.DRIVING or self._safety_stopped:
            return
        if self._cmdvel_zero_since is None:
            return
        stil_sec = (self.get_clock().now() - self._cmdvel_zero_since).nanoseconds / 1e9
        if stil_sec > 15.0:
            self._safety_stopped = True
            self._publish_state()
            self.get_logger().info(
                'EZ-Wheel veiligheidsstop actief — robot gestopt door persoon/object')
            if self._safety_wait_timer is None and not self._backing_up:
                self._safety_wait_timer = self.create_timer(
                    PERSOON_WACHT_SECONDEN, self._on_person_wait_voorbij)

    def _check_nav_timeout(self):
        """Per-waypoint timeout: _driving_since wordt gereset via _on_feedback
        telkens als een waypoint gepasseerd wordt."""
        if self._backing_up:
            return
        if self._state != State.DRIVING or self._driving_since is None:
            return
        elapsed = (self.get_clock().now() - self._driving_since).nanoseconds / 1e9
        if elapsed > NAV_TIMEOUT_SEC:
            self.get_logger().error(
                f'Nav2 timeout na {NAV_TIMEOUT_SEC}s — geen voortgang bij '
                f'waypoint {self._current_index + 1}')
            self._driving_since = None
            self._on_blocked()

    # ── Achteruit rijden ──────────────────────────────────────────────────────

    def _cancel_safety_wait_timer(self):
        if self._safety_wait_timer is not None:
            self._safety_wait_timer.cancel()
            self._safety_wait_timer = None

    def _on_person_wait_voorbij(self):
        self._cancel_safety_wait_timer()
        if self._stopped or not self._safety_stopped:
            return
        self.get_logger().warn(
            f'Persoon blokkeert al {PERSOON_WACHT_SECONDEN // 60} min — escaleren')
        self._escalate_blockage()

    def _start_backup(self):
        self._backing_up   = True
        self._backup_start = self.get_clock().now()
        self._backup_timer = self.create_timer(0.1, self._backup_step)

    def _backup_step(self):
        duur       = BACKUP_DISTANCE / BACKUP_SPEED
        verstreken = (self.get_clock().now() - self._backup_start).nanoseconds / 1e9
        twist      = Twist()
        if verstreken < duur and not self._stopped:
            twist.linear.x = -BACKUP_SPEED
            self._cmdvel_pub.publish(twist)
            return
        twist.linear.x = 0.0
        self._cmdvel_pub.publish(twist)
        if self._backup_timer is not None:
            self._backup_timer.cancel()
            self._backup_timer = None
        self._backing_up = False
        if not self._stopped:
            self.get_logger().info('Achteruit klaar — route hervatten')
            self._navigate_through_remaining()

    # ── Hulpfuncties ──────────────────────────────────────────────────────────

    def _make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        return pose


def _raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


def main():
    rclpy.init()
    node = PatrolNode()
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_cleanly()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
