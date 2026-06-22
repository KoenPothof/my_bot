"""Unit tests voor de state machine van patrol_node.

Aanpak: de PatrolNode wordt echt geïnstantieerd, maar de ROS-I/O (ActionClient,
CancelGoal-client) is gemockt (zie conftest.py). We roepen de callbacks direct aan
met nep-berichten/-futures en controleren de state en de zijeffecten.
"""
from unittest.mock import MagicMock

from rclpy.duration import Duration
from std_msgs.msg import Bool
from action_msgs.msg import GoalStatus

from patrol_node import State, NAV_TIMEOUT_SEC


# ── Hulpjes ──────────────────────────────────────────────────────────────────

def _bool(value):
    msg = Bool()
    msg.data = value
    return msg


def _result_future(status):
    """Nep-future zoals get_result_async() teruggeeft (heeft .result().status)."""
    fut = MagicMock()
    fut.result.return_value.status = status
    return fut


def _goal_response_future(accepted):
    """Nep-future zoals send_goal_async() teruggeeft (heeft .result().accepted)."""
    fut = MagicMock()
    fut.result.return_value.accepted = accepted
    return fut


# ── Basis ────────────────────────────────────────────────────────────────────

def test_start_state_is_idle(node):
    assert node._state == State.IDLE


# ── _on_start: bewaking & reset ───────────────────────────────────────────────

def test_start_ignored_when_driving(node):
    node._cancel_all_nav2_goals = MagicMock()
    node._state = State.DRIVING
    node._on_start(_bool(True))
    assert node._state == State.DRIVING
    node._cancel_all_nav2_goals.assert_not_called()


def test_start_ignored_when_waiting(node):
    node._cancel_all_nav2_goals = MagicMock()
    node._state = State.WAITING
    node._on_start(_bool(True))
    node._cancel_all_nav2_goals.assert_not_called()


def test_start_ignored_when_msg_false(node):
    node._cancel_all_nav2_goals = MagicMock()
    node._on_start(_bool(False))
    node._cancel_all_nav2_goals.assert_not_called()


def test_start_from_idle_resets_and_cancels(node):
    node._cancel_all_nav2_goals = MagicMock()
    node._current_index = 3
    node._last_successful_idx = 2
    node._trying_alternative = True
    node._stopped = True
    node._on_start(_bool(True))
    assert node._current_index == 0
    assert node._last_successful_idx == 0
    assert node._trying_alternative is False
    assert node._stopped is False
    node._cancel_all_nav2_goals.assert_called_once()


def test_start_allowed_from_completed(node):
    node._cancel_all_nav2_goals = MagicMock()
    node._state = State.COMPLETED
    node._on_start(_bool(True))
    node._cancel_all_nav2_goals.assert_called_once()


def test_start_resumes_from_saved_progress(node):
    node._cancel_all_nav2_goals = MagicMock()
    node._load_progress.return_value = 1
    node._on_start(_bool(True))
    assert node._current_index == 1
    assert node._last_successful_idx == 1


# ── Navigatie / DRIVING ───────────────────────────────────────────────────────

def test_navigate_sets_driving_and_sends_goal(node):
    node._navigate_to_current()
    assert node._state == State.DRIVING
    node._action_client.send_goal_async.assert_called_once()


def test_navigate_error_when_nav2_unavailable(node):
    node._action_client.wait_for_server.return_value = False
    node._navigate_to_current()
    assert node._state == State.ERROR


# ── _on_goal_response ─────────────────────────────────────────────────────────

def test_goal_rejected_first_time_retries(node):
    node._schedule_start_navigation = MagicMock()
    node._on_blocked = MagicMock()
    node._goal_retries = 0
    node._on_goal_response(_goal_response_future(accepted=False))
    node._schedule_start_navigation.assert_called_once()
    node._on_blocked.assert_not_called()


def test_goal_rejected_triggers_blocked_after_retry(node):
    node._on_blocked = MagicMock()
    node._goal_retries = 1  # retry al verbruikt → nu als blokkade behandelen
    node._on_goal_response(_goal_response_future(accepted=False))
    node._on_blocked.assert_called_once()


# ── _on_result ────────────────────────────────────────────────────────────────

def test_result_success_advances_to_next_waypoint(node):
    node._navigate_to_current = MagicMock()
    node._current_index = 0
    node._on_result(_result_future(GoalStatus.STATUS_SUCCEEDED))
    assert node._last_successful_idx == 0
    assert node._current_index == 1
    node._navigate_to_current.assert_called_once()


def test_result_success_last_waypoint_completes(node):
    node._navigate_to_current = MagicMock()
    node._current_index = len(node._waypoints) - 1
    node._on_result(_result_future(GoalStatus.STATUS_SUCCEEDED))
    assert node._state == State.COMPLETED
    node._navigate_to_current.assert_not_called()


def test_result_failure_triggers_blocked(node):
    node._on_blocked = MagicMock()
    node._on_result(_result_future(GoalStatus.STATUS_ABORTED))
    node._on_blocked.assert_called_once()


def test_result_ignored_when_stopped(node):
    node._stopped = True
    node._on_blocked = MagicMock()
    node._state = State.STOPPED
    node._on_result(_result_future(GoalStatus.STATUS_ABORTED))
    node._on_blocked.assert_not_called()
    assert node._state == State.STOPPED


def test_result_success_saves_progress(node):
    node._navigate_to_current = MagicMock()
    node._current_index = 0
    node._on_result(_result_future(GoalStatus.STATUS_SUCCEEDED))
    node._save_progress.assert_called_once_with(1)


def test_result_complete_clears_progress(node):
    node._navigate_to_current = MagicMock()
    node._current_index = len(node._waypoints) - 1
    node._on_result(_result_future(GoalStatus.STATUS_SUCCEEDED))
    node._clear_progress.assert_called_once()


def test_result_success_resets_blockage_level(node):
    node._navigate_to_current = MagicMock()
    node._blockage_level = 2
    node._current_index = 0
    node._on_result(_result_future(GoalStatus.STATUS_SUCCEEDED))
    assert node._blockage_level == 0


# ── Blokkade-afhandeling ──────────────────────────────────────────────────────

def test_blocked_level0_waits(node):
    node._blockage_level = 0
    node._on_blocked()
    assert node._state == State.WAITING
    assert node._wait_timer is not None
    node._cancel_wait_timer()


def test_blocked_nonzero_level_escalates(node):
    node._escalate_blockage = MagicMock()
    node._blockage_level = 1
    node._on_blocked()
    node._escalate_blockage.assert_called_once()


def test_wacht_voorbij_escalates(node):
    node._escalate_blockage = MagicMock()
    node._on_wacht_voorbij()
    node._escalate_blockage.assert_called_once()


def test_escalate_level0_beeps_and_backs_up(node):
    node._cancel_goal = MagicMock()
    node._beep = MagicMock()
    node._start_backup = MagicMock()
    node._blockage_level = 0
    node._escalate_blockage()
    node._beep.assert_called_once()
    node._start_backup.assert_called_once()
    assert node._blockage_level == 1


def test_escalate_level1_tries_alternative(node):
    node._navigate_to_current = MagicMock()
    node._blockage_level = 1
    node._current_index = 0
    node._escalate_blockage()
    assert node._trying_alternative is True
    assert node._current_index == 1
    assert node._blockage_level == 2
    node._navigate_to_current.assert_called_once()


def test_escalate_level1_no_more_waypoints_returns(node):
    node._return_to_last = MagicMock()
    node._navigate_to_current = MagicMock()
    node._blockage_level = 1
    node._current_index = len(node._waypoints) - 1
    node._escalate_blockage()
    node._return_to_last.assert_called_once()
    node._navigate_to_current.assert_not_called()


def test_escalate_level2_returns_to_last(node):
    node._return_to_last = MagicMock()
    node._blockage_level = 2
    node._escalate_blockage()
    node._return_to_last.assert_called_once()


# ── Stop ──────────────────────────────────────────────────────────────────────

def test_stop_transitions_to_stopped_and_cancels_goal(node):
    node._state = State.DRIVING
    node._goal_handle = MagicMock()
    node._on_stop(_bool(True))
    assert node._state == State.STOPPED
    assert node._stopped is True
    assert node._goal_handle is None


def test_stop_ignored_when_idle(node):
    node._state = State.IDLE
    node._on_stop(_bool(True))
    assert node._state == State.IDLE


# ── Veiligheidsstop (ez-wheel) ────────────────────────────────────────────────

def test_publish_state_reports_waiting_when_safety_stopped(node):
    node._state = State.DRIVING
    node._safety_stopped = True
    node._state_pub.publish = MagicMock()
    node._publish_state()
    sent = node._state_pub.publish.call_args[0][0]
    assert sent.data == State.WAITING


def test_check_safety_stop_sets_flag_after_2s(node):
    node._state = State.DRIVING
    node._was_moving = True
    node._cmdvel_zero_since = node.get_clock().now() - Duration(seconds=3)
    node._check_safety_stop()
    assert node._safety_stopped is True


def test_check_safety_stop_not_triggered_too_soon(node):
    node._state = State.DRIVING
    node._cmdvel_zero_since = node.get_clock().now() - Duration(seconds=1)
    node._check_safety_stop()
    assert node._safety_stopped is False


# ── Nav2 timeout-bewaker ──────────────────────────────────────────────────────

def test_nav_timeout_triggers_blocked(node):
    node._on_blocked = MagicMock()
    node._state = State.DRIVING
    node._driving_since = node.get_clock().now() - Duration(seconds=NAV_TIMEOUT_SEC + 1)
    node._check_nav_timeout()
    node._on_blocked.assert_called_once()
    assert node._driving_since is None


def test_nav_timeout_not_triggered_early(node):
    node._on_blocked = MagicMock()
    node._state = State.DRIVING
    node._driving_since = node.get_clock().now() - Duration(seconds=5)
    node._check_nav_timeout()
    node._on_blocked.assert_not_called()


# ── Wacht-en-hervat bij langdurige persoon-detectie ───────────────────────────

def test_person_wait_escalates(node):
    node._escalate_blockage = MagicMock()
    node._safety_stopped = True
    node._on_person_wait_voorbij()
    node._escalate_blockage.assert_called_once()


def test_person_wait_skips_when_not_stopped(node):
    node._escalate_blockage = MagicMock()
    node._safety_stopped = False
    node._on_person_wait_voorbij()
    node._escalate_blockage.assert_not_called()


def test_backup_finished_resumes_navigation(node):
    node._navigate_to_current = MagicMock()
    node._backing_up = True
    node._backup_timer = MagicMock()
    # Zet de starttijd ver genoeg terug zodat de achteruit-afstand 'bereikt' is.
    node._backup_start = node.get_clock().now() - Duration(seconds=60)
    node._backup_step()
    assert node._backing_up is False
    node._navigate_to_current.assert_called_once()
