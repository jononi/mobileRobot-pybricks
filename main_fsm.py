"""Remote-controlled target navigation with reactive obstacle avoidance."""

import math


class NavigationFSM:
    IDLE = "IDLE"
    GO_TO_TARGET = "GO_TO_TARGET"
    TURN_CLEAR = "TURN_CLEAR"
    BYPASS = "BYPASS"
    BLOCKED = "BLOCKED"

    OBSTACLE_MM = 300
    CLEAR_MM = 400
    CLEAR_SAMPLES = 3
    TARGET_TOLERANCE_M = 0.03
    TARGET_SPEED_M_S = 0.15
    BYPASS_SPEED_M_S = 0.10
    BYPASS_DISTANCE_M = 0.40
    TURN_SPEED_RAD_S = math.radians(30)
    TURN_LIMIT_RAD = math.radians(90)
    TURN_TIMEOUT_MS = 8000
    BYPASS_TIMEOUT_MS = 10000
    ATTEMPT_TIMEOUT_MS = 120000
    PROGRESS_TIMEOUT_MS = 20000
    PROGRESS_STEP_M = 0.05
    MAX_DETOURS = 12

    def __init__(self, model, turn_sign=1):
        if turn_sign not in (-1, 1):
            raise ValueError("turn_sign must be -1 (left) or 1 (right)")
        self.model = model
        self.turn_sign = turn_sign
        self.state = self.IDLE
        self.reason = "Ready"
        self.state_started_ms = 0
        self.attempt_started_ms = 0
        self.progress_ms = 0
        self.best_distance_m = 0.0
        self.detours = 0
        self.clear_count = 0
        self.turn_start_heading = 0.0
        self.bypass_heading = 0.0
        self.bypass_start_m = 0.0

    @property
    def moving(self):
        return self.state in (self.GO_TO_TARGET, self.TURN_CLEAR, self.BYPASS)

    def _enter(self, state, now_ms, distance_m, reason):
        self.state = state
        self.reason = reason
        self.state_started_ms = now_ms
        if state == self.TURN_CLEAR:
            self.turn_start_heading = self.model.theta
            self.clear_count = 0
        elif state == self.BYPASS:
            self.bypass_start_m = distance_m
            self.bypass_heading = self.model.theta
        print("{}: {}".format(state, reason))
        return 0.0, 0.0

    def _avoid(self, now_ms, distance_m):
        self.detours += 1
        if self.detours > self.MAX_DETOURS:
            return self._enter(self.BLOCKED, now_ms, distance_m,
                               "Too many avoidance maneuvers")
        return self._enter(self.TURN_CLEAR, now_ms, distance_m,
                           "Obstacle ahead")

    def update(self, now_ms, distance_m, obstacle_mm, connected=True,
               start_target=None, cancel=False, acknowledge=False):
        """Return (m/s, rad/s); callers must update pose before each tick."""
        if cancel or (self.moving and not connected):
            return self._enter(self.IDLE, now_ms, distance_m,
                               "Cancelled or remote disconnected")

        if self.state == self.IDLE:
            if start_target is not None and connected:
                self.model.set_target(*start_target)
                self.best_distance_m, _ = self.model.get_course_to_target()
                self.attempt_started_ms = now_ms
                self.progress_ms = now_ms
                self.detours = 0
                return self._enter(self.GO_TO_TARGET, now_ms, distance_m,
                                   "Target = {}".format(start_target))
            return 0.0, 0.0

        if self.state == self.BLOCKED:
            if acknowledge and connected:
                return self._enter(self.IDLE, now_ms, distance_m,
                                   "Failure acknowledged; edit or retry target")
            return 0.0, 0.0

        if obstacle_mm is None or not 0 <= obstacle_mm <= 2000:
            return self._enter(self.BLOCKED, now_ms, distance_m,
                               "Ultrasonic sensor unavailable")

        distance_to_target, alpha = self.model.get_course_to_target()
        if distance_to_target < self.TARGET_TOLERANCE_M:
            return self._enter(self.IDLE, now_ms, distance_m, "Target reached")
        if distance_to_target <= self.best_distance_m - self.PROGRESS_STEP_M:
            self.best_distance_m = distance_to_target
            self.progress_ms = now_ms
        if now_ms - self.attempt_started_ms >= self.ATTEMPT_TIMEOUT_MS:
            return self._enter(self.BLOCKED, now_ms, distance_m,
                               "Attempt timed out")
        if now_ms - self.progress_ms >= self.PROGRESS_TIMEOUT_MS:
            return self._enter(self.BLOCKED, now_ms, distance_m,
                               "No progress toward target")

        if self.state == self.GO_TO_TARGET:
            if obstacle_mm < self.OBSTACLE_MM:
                return self._avoid(now_ms, distance_m)
            velocity, angular_velocity = self.model.update_controls(kv=1.2, kw=2.0)
            if abs(alpha) > math.radians(45):
                velocity = 0.0
            return max(0.0, min(self.TARGET_SPEED_M_S, velocity)), max(
                -1.0, min(1.0, angular_velocity))

        if self.state == self.TURN_CLEAR:
            turn_angle = abs(self.model.theta - self.turn_start_heading)
            if (turn_angle >= self.TURN_LIMIT_RAD or
                    now_ms - self.state_started_ms >= self.TURN_TIMEOUT_MS):
                return self._enter(self.BLOCKED, now_ms, distance_m,
                                   "No clear heading within turn limit")
            self.clear_count = self.clear_count + 1 if obstacle_mm > self.CLEAR_MM else 0
            if self.clear_count >= self.CLEAR_SAMPLES:
                return self._enter(self.BYPASS, now_ms, distance_m,
                                   "Forward clearance found")
            return 0.0, self.turn_sign * self.TURN_SPEED_RAD_S

        if self.state == self.BYPASS:
            if obstacle_mm < self.OBSTACLE_MM:
                return self._avoid(now_ms, distance_m)
            if now_ms - self.state_started_ms >= self.BYPASS_TIMEOUT_MS:
                return self._enter(self.BLOCKED, now_ms, distance_m,
                                   "Bypass timed out")
            if distance_m - self.bypass_start_m >= self.BYPASS_DISTANCE_M:
                return self._enter(self.GO_TO_TARGET, now_ms, distance_m,
                                   "Retry original target")
            heading_error = self.bypass_heading - self.model.theta
            heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi
            return self.BYPASS_SPEED_M_S, max(-1.0, min(1.0, 2.0 * heading_error))

        return self._enter(self.BLOCKED, now_ms, distance_m, "Unknown state")


class IncrementalRemoteUI:
    STEP_M = 0.1
    CHARACTER_MS = 250

    def __init__(self, hub, button, remote_factory, target_x=2.0, target_y=0.0):
        self.hub = hub
        self.button = button
        self.remote_factory = remote_factory
        self.remote = None
        self.previous = set()
        self.x_steps = round(target_x / self.STEP_M)
        self.y_steps = round(target_y / self.STEP_M)
        self.next_connect_ms = 0
        self.message = ""
        self.next_character_ms = 0
        self.status = "I"

    def show_target(self, axis):
        steps = self.x_steps if axis == "X" else self.y_steps
        self.message = "{} {:.1f} ".format(axis, steps * self.STEP_M)
        self.next_character_ms = 0
        print("Target: ({:.1f}, {:.1f}) m".format(*self.target))

    @property
    def target(self):
        return self.x_steps * self.STEP_M, self.y_steps * self.STEP_M

    def set_status(self, status):
        self.status = status
        self.message = ""
        self.hub.display.char(status)

    def poll(self, now_ms, allow_connect):
        if self.remote is None:
            self.hub.display.char("C")
            if allow_connect and now_ms >= self.next_connect_ms:
                try:
                    self.remote = self.remote_factory(timeout=1000)
                    self.previous = set(self.remote.buttons.pressed())
                    self.show_target("X")
                except OSError:
                    self.remote = None
                self.next_connect_ms = now_ms + 2000
            return set()
        try:
            pressed = set(self.remote.buttons.pressed())
        except OSError:
            self.remote = None
            self.previous = set()
            return set()
        new_buttons = pressed - self.previous
        self.previous = pressed
        return new_buttons

    def edit(self, new_buttons):
        x_delta = (int(self.button.LEFT_PLUS in new_buttons) -
                   int(self.button.LEFT_MINUS in new_buttons))
        y_delta = (int(self.button.RIGHT_PLUS in new_buttons) -
                   int(self.button.RIGHT_MINUS in new_buttons))
        self.x_steps += x_delta
        self.y_steps += y_delta
        if x_delta or self.button.LEFT in new_buttons:
            self.show_target("X")
        if y_delta or self.button.RIGHT in new_buttons:
            self.show_target("Y")

    def update_display(self, now_ms):
        if self.remote is None or now_ms < self.next_character_ms:
            return
        character = self.message[0] if self.message else self.status
        self.hub.display.char(character)
        self.message = self.message[1:]
        self.next_character_ms = now_ms + self.CHARACTER_MS


def main():
    from pybricks.hubs import InventorHub
    from pybricks.parameters import Button, Port
    from pybricks.pupdevices import Remote, UltrasonicSensor
    from pybricks.tools import StopWatch, wait
    from main import MobileModeling, MobileRobot

    hub = InventorHub()
    robot = MobileRobot(hub)
    try:
        robot.reset()
        sensor = UltrasonicSensor(Port.C)
        model = MobileModeling()
        navigation = NavigationFSM(model, turn_sign=1)
        ui = IncrementalRemoteUI(hub, Button, Remote)
        timer = StopWatch()
        last_print_ms = -500
        status_chars = {
            navigation.IDLE: "I", navigation.GO_TO_TARGET: "G",
            navigation.TURN_CLEAR: "T", navigation.BYPASS: "B",
            navigation.BLOCKED: "!",
        }
        while True:
            distance_mm, speed_mm_s, angle_deg, turn_rate_deg_s = robot.get_driving_state()
            model.update_pose(distance_mm, angle_deg)
            new_buttons = ui.poll(timer.time(), allow_connect=not navigation.moving)
            now_ms = timer.time()
            center_pressed = Button.CENTER in new_buttons
            start_target = None
            if navigation.state == navigation.IDLE:
                ui.edit(new_buttons)
                if center_pressed:
                    start_target = ui.target

            obstacle_mm = None
            if navigation.moving and ui.remote is not None and not center_pressed:
                try:
                    obstacle_mm = sensor.distance()
                except OSError:
                    pass

            now_ms = timer.time()
            previous_state = navigation.state
            velocity, angular_velocity = navigation.update(
                now_ms, distance_mm / 1000.0, obstacle_mm,
                connected=ui.remote is not None, start_target=start_target,
                cancel=center_pressed and navigation.moving,
                acknowledge=center_pressed and navigation.state == navigation.BLOCKED,
            )
            if velocity == 0.0 and angular_velocity == 0.0:
                robot.stop()
            else:
                robot.apply_controls(velocity, angular_velocity)
            if navigation.state != previous_state:
                ui.set_status(status_chars[navigation.state])
            ui.update_display(now_ms)

            if now_ms - last_print_ms >= 500:
                print("[{}] x={:.3f} y={:.3f} heading={:.1f} range={} mm".format(
                    navigation.state, model.x, model.y,
                    math.degrees(model.theta), obstacle_mm))
                last_print_ms = now_ms
            wait(40)
    finally:
        robot.stop()


if __name__ == "__main__":
    main()