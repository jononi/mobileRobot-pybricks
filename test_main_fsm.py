"""Hardware-free checks: python -m unittest -v test_main_fsm."""

import math
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from main_fsm import IncrementalRemoteUI, NavigationFSM


class NavigationTests(unittest.TestCase):
    def setUp(self):
        self.output = patch("builtins.print")
        self.output.start()
        self.addCleanup(self.output.stop)
        self.model = Mock(theta=0.0)
        self.model.get_course_to_target.return_value = (5.0, 0.0)
        self.model.update_controls.return_value = (1.0, 0.0)
        self.fsm = NavigationFSM(self.model)

    def start(self):
        self.assertEqual(self.fsm.update(0, 0, None, start_target=(5, 0)), (0, 0))
        self.assertEqual(self.fsm.state, self.fsm.GO_TO_TARGET)

    def begin_turn(self):
        self.start()
        self.assertEqual(self.fsm.update(40, 0, 299), (0, 0))
        self.assertEqual(self.fsm.state, self.fsm.TURN_CLEAR)

    def begin_bypass(self):
        self.begin_turn()
        for now_ms in (80, 120, 160):
            self.fsm.update(now_ms, 0, 500)
        self.assertEqual(self.fsm.state, self.fsm.BYPASS)

    def test_idle_requires_confirmation_and_connection(self):
        self.assertEqual(self.fsm.update(0, 0, None), (0, 0))
        self.fsm.update(40, 0, None, connected=False, start_target=(5, 0))
        self.assertEqual(self.fsm.state, self.fsm.IDLE)
        self.model.set_target.assert_not_called()

    def test_forward_speed_and_turn_rate_are_limited(self):
        self.start()
        self.model.update_controls.return_value = (2.0, 3.0)
        self.assertEqual(self.fsm.update(40, 0, 500), (0.15, 1.0))

    def test_target_behind_robot_never_causes_reverse(self):
        self.start()
        self.model.get_course_to_target.return_value = (5.0, math.pi)
        self.model.update_controls.return_value = (-1.0, 3.0)
        self.assertEqual(self.fsm.update(40, 0, 500), (0, 1.0))

    def test_clearance_requires_consecutive_samples_above_exit_threshold(self):
        self.begin_turn()
        for now_ms, reading in ((80, 500), (120, 400), (160, 500), (200, 500)):
            self.fsm.update(now_ms, 0, reading)
            self.assertEqual(self.fsm.state, self.fsm.TURN_CLEAR)
        self.assertEqual(self.fsm.update(240, 0, 500), (0, 0))
        self.assertEqual(self.fsm.state, self.fsm.BYPASS)

    def test_entry_threshold_is_strictly_below_300_mm(self):
        self.start()
        self.fsm.update(40, 0, 300)
        self.assertEqual(self.fsm.state, self.fsm.GO_TO_TARGET)

    def test_turn_direction_is_configurable(self):
        self.fsm = NavigationFSM(self.model, turn_sign=-1)
        self.begin_turn()
        self.assertEqual(self.fsm.update(80, 0, 200),
                         (0, -self.fsm.TURN_SPEED_RAD_S))
        with self.assertRaises(ValueError):
            NavigationFSM(self.model, turn_sign=0)

    def test_turn_angle_limit_stops_even_when_range_is_clear(self):
        self.begin_turn()
        self.model.theta = self.fsm.TURN_LIMIT_RAD
        self.assertEqual(self.fsm.update(80, 0, 500), (0, 0))
        self.assertEqual(self.fsm.state, self.fsm.BLOCKED)

    def test_turn_timeout(self):
        self.begin_turn()
        self.assertEqual(self.fsm.update(8040, 0, 200), (0, 0))
        self.assertEqual(self.fsm.state, self.fsm.BLOCKED)

    def test_bypass_holds_heading_and_resumes_original_target(self):
        self.begin_bypass()
        self.model.theta = 0.1
        velocity, angular_velocity = self.fsm.update(200, 0.1, 500)
        self.assertEqual(velocity, 0.1)
        self.assertAlmostEqual(angular_velocity, -0.2)
        self.assertEqual(self.fsm.update(240, 0.41, 500), (0, 0))
        self.assertEqual(self.fsm.state, self.fsm.GO_TO_TARGET)
        self.model.set_target.assert_called_once_with(5, 0)

    def test_bypass_distance_is_relative_to_state_entry(self):
        self.begin_turn()
        for now_ms in (80, 120, 160):
            self.fsm.update(now_ms, 2.0, 500)
        self.assertEqual(self.fsm.update(200, 2.2, 500)[0], 0.1)
        self.fsm.update(240, 2.41, 500)
        self.assertEqual(self.fsm.state, self.fsm.GO_TO_TARGET)

    def test_obstacle_during_bypass_starts_another_turn(self):
        self.begin_bypass()
        self.assertEqual(self.fsm.update(200, 0.1, 200), (0, 0))
        self.assertEqual(self.fsm.state, self.fsm.TURN_CLEAR)
        self.assertEqual(self.fsm.detours, 2)
        self.assertEqual(self.fsm.clear_count, 0)

    def test_bypass_timeout(self):
        self.begin_bypass()
        self.assertEqual(self.fsm.update(10160, 0.1, 500), (0, 0))
        self.assertEqual(self.fsm.state, self.fsm.BLOCKED)

    def test_stop_conditions_override_all_moving_states(self):
        for state in (self.fsm.GO_TO_TARGET, self.fsm.TURN_CLEAR, self.fsm.BYPASS):
            for stop_input in ({"connected": False}, {"cancel": True}):
                with self.subTest(state=state, stop_input=stop_input):
                    self.fsm.state = state
                    self.assertEqual(self.fsm.update(40, 0, None, **stop_input), (0, 0))
                    self.assertEqual(self.fsm.state, self.fsm.IDLE)

    def test_sensor_faults_stop_and_require_acknowledgement(self):
        for reading in (None, -1, 2001):
            with self.subTest(reading=reading):
                self.fsm = NavigationFSM(self.model)
                self.start()
                self.assertEqual(self.fsm.update(40, 0, reading), (0, 0))
                self.assertEqual(self.fsm.state, self.fsm.BLOCKED)
                self.fsm.update(80, 0, 500, start_target=(1, 0))
                self.assertEqual(self.fsm.state, self.fsm.BLOCKED)
                self.fsm.update(120, 0, 500, acknowledge=True)
                self.assertEqual(self.fsm.state, self.fsm.IDLE)

    def test_sensor_no_echo_value_is_accepted(self):
        self.start()
        self.assertEqual(self.fsm.update(40, 0, 2000)[0], 0.15)

    def test_target_reached_stops_from_any_moving_state(self):
        for state in (self.fsm.GO_TO_TARGET, self.fsm.TURN_CLEAR, self.fsm.BYPASS):
            with self.subTest(state=state):
                self.fsm.state = state
                self.model.get_course_to_target.return_value = (0.02, 0)
                self.assertEqual(self.fsm.update(40, 0, 500), (0, 0))
                self.assertEqual(self.fsm.state, self.fsm.IDLE)

    def test_attempt_timeout_is_not_reset_by_detours(self):
        self.begin_bypass()
        self.assertEqual(self.fsm.attempt_started_ms, 0)
        self.assertEqual(self.fsm.update(120000, 0.1, 500), (0, 0))
        self.assertEqual(self.fsm.reason, "Attempt timed out")

    def test_no_progress_watchdog_and_progress_refresh(self):
        self.start()
        self.model.get_course_to_target.return_value = (4.9, 0)
        self.fsm.update(19000, 0.1, 500)
        self.assertEqual(self.fsm.progress_ms, 19000)
        self.fsm.update(20000, 0.1, 500)
        self.assertEqual(self.fsm.state, self.fsm.GO_TO_TARGET)
        self.assertEqual(self.fsm.update(39000, 0.1, 500), (0, 0))
        self.assertEqual(self.fsm.reason, "No progress toward target")

    def test_detour_limit_and_new_attempt_reset(self):
        self.start()
        self.fsm.detours = self.fsm.MAX_DETOURS
        self.assertEqual(self.fsm.update(40, 0, 200), (0, 0))
        self.assertEqual(self.fsm.state, self.fsm.BLOCKED)
        self.fsm.update(80, 0, None, acknowledge=True)
        self.fsm.update(120, 0, None, start_target=(5, 0))
        self.assertEqual(self.fsm.detours, 0)
        self.assertEqual(self.fsm.attempt_started_ms, 120)


class RemoteUITests(unittest.TestCase):
    def setUp(self):
        self.output = patch("builtins.print")
        self.output.start()
        self.addCleanup(self.output.stop)
        self.hub = Mock()
        self.button = SimpleNamespace(
            CENTER="center", LEFT_PLUS="x+", LEFT_MINUS="x-",
            RIGHT_PLUS="y+", RIGHT_MINUS="y-", LEFT="x", RIGHT="y")
        self.remote = Mock()
        self.remote.buttons.pressed.return_value = set()
        self.factory = Mock(return_value=self.remote)
        self.ui = IncrementalRemoteUI(self.hub, self.button, self.factory)

    def test_connection_is_only_attempted_while_stopped(self):
        self.ui.poll(0, allow_connect=False)
        self.factory.assert_not_called()
        self.ui.poll(40, allow_connect=True)
        self.factory.assert_called_once_with(timeout=1000)

    def test_connection_failure_is_retried_with_backoff(self):
        self.factory.side_effect = OSError("not connected")
        self.ui.poll(0, allow_connect=True)
        self.ui.poll(40, allow_connect=True)
        self.assertEqual(self.factory.call_count, 1)
        self.ui.poll(2000, allow_connect=True)
        self.assertEqual(self.factory.call_count, 2)
        self.assertIsNone(self.ui.remote)

    def test_held_green_needs_release_before_new_confirmation(self):
        self.remote.buttons.pressed.return_value = {self.button.CENTER}
        self.assertEqual(self.ui.poll(0, True), set())
        self.assertEqual(self.ui.poll(40, True), set())
        self.remote.buttons.pressed.return_value = set()
        self.ui.poll(80, True)
        self.remote.buttons.pressed.return_value = {self.button.CENTER}
        self.assertEqual(self.ui.poll(120, True), {self.button.CENTER})
        self.assertEqual(self.ui.poll(160, True), set())

    def test_disconnect_is_exposed_to_navigation(self):
        self.ui.poll(0, True)
        self.remote.buttons.pressed.side_effect = OSError("disconnected")
        self.assertEqual(self.ui.poll(40, False), set())
        self.assertIsNone(self.ui.remote)

    def test_target_editing_and_nonblocking_display(self):
        self.ui.poll(0, True)
        self.ui.edit({self.button.LEFT_PLUS, self.button.RIGHT_MINUS})
        self.assertEqual(self.ui.target, (2.1, -0.1))
        self.ui.update_display(40)
        self.hub.display.char.assert_called_with("Y")
        calls = self.hub.display.char.call_count
        self.ui.update_display(80)
        self.assertEqual(self.hub.display.char.call_count, calls)
        self.ui.set_status("G")
        self.assertEqual(self.ui.message, "")
        self.hub.display.char.assert_called_with("G")


if __name__ == "__main__":
    unittest.main()