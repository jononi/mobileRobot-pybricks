"""
    Mobile Differential Robot based on Pybricks 4.1.0
    Dead Reckoning Benchmark & Lyapunov Control with Periodic Telemetry
"""

import math
from pybricks.hubs import InventorHub
from pybricks.pupdevices import Motor, Remote
from pybricks.parameters import Port, Direction, Button
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, wait

class RemoteTargetUI:
    """Select absolute targets in 0.1 m steps with a Powered Up remote."""

    STEP_M = 0.1

    def __init__(self, hub: InventorHub, target_x=2.0, target_y=0.0):
        self.hub = hub
        self.remote = None
        self.x_steps = round(target_x / self.STEP_M)
        self.y_steps = round(target_y / self.STEP_M)

    def connect(self):
        self.hub.display.char("C")
        while self.remote is None:
            try:
                self.remote = Remote(timeout=1000)
            except OSError:
                wait(100)

    def _pressed(self):
        try:
            return self.remote.buttons.pressed()
        except OSError:
            self.remote = None
            return None

    def _show_target(self, axis):
        steps = self.x_steps if axis == "X" else self.y_steps
        self.hub.display.char(axis)
        wait(300)
        self.hub.display.text("{:.1f}".format(steps * self.STEP_M), 250, 50)

    def get_target(self):
        """Wait for edits and green-button confirmation while the robot is stopped."""
        if self.remote is not None:
            self._show_target("X")
            self._show_target("Y")
        previous = self._pressed() if self.remote is not None else set()
        while True:
            if self.remote is None:
                self.connect()
                self._show_target("X")
                self._show_target("Y")
                previous = self._pressed()
                if previous is None:
                    continue

            pressed = self._pressed()
            if pressed is None:
                continue
            new_buttons = pressed - previous
            previous = pressed

            if Button.CENTER in new_buttons:
                while Button.CENTER in pressed:
                    wait(40)
                    pressed = self._pressed()
                    if pressed is None:
                        break
                if pressed is None:
                    continue
                self.hub.display.char("G")
                return self.x_steps * self.STEP_M, self.y_steps * self.STEP_M

            x_delta = int(Button.LEFT_PLUS in new_buttons) - int(Button.LEFT_MINUS in new_buttons)
            y_delta = int(Button.RIGHT_PLUS in new_buttons) - int(Button.RIGHT_MINUS in new_buttons)
            self.x_steps += x_delta
            self.y_steps += y_delta
            if x_delta or Button.LEFT in new_buttons:
                self._show_target("X")
            if y_delta or Button.RIGHT in new_buttons:
                self._show_target("Y")
            wait(40)

    def stop_requested(self):
        """A disconnected remote or green button cancels the current attempt."""
        if self.remote is None:
            return True
        pressed = self._pressed()
        return pressed is None or Button.CENTER in pressed


class MobileRobot:
    Wheel_diameter = 88  # mm
    Wheel_base = 146     # mm

    # Velocity saturation bounds derived from ~800 deg/s motor speed limit
    MAX_V = 0.40  # m/s
    MAX_W = 4.00  # rad/s

    def __init__(self, hub: InventorHub):
        self.hub = hub
        self.left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
        self.right_motor = Motor(Port.B)
        self.drive_base = DriveBase(
            self.left_motor, 
            self.right_motor, 
            wheel_diameter=self.Wheel_diameter, 
            axle_track=self.Wheel_base
        )
        self.drive_base.use_gyro(True)
        self.drive_base.settings(
            straight_speed=300, 
            straight_acceleration=1000, 
            turn_rate=230, 
            turn_acceleration=1000
        )

    def reset(self):
        self.left_motor.reset_angle(0)
        self.right_motor.reset_angle(0)
        self.drive_base.reset()

    def apply_controls(self, v_m_s: float, w_rad_s: float):
        """Clamps velocity targets to physical limits and commands DriveBase."""
        v_clamped = max(-self.MAX_V, min(self.MAX_V, v_m_s))
        w_clamped = max(-self.MAX_W, min(self.MAX_W, w_rad_s))

        linear_mm_s = v_clamped * 1000.0
        angular_deg_s = math.degrees(w_clamped)

        self.drive_base.drive(speed=linear_mm_s, turn_rate=angular_deg_s)

    def stop(self):
        self.drive_base.stop()

    def get_driving_state(self):
        #{distance, lin. speed (v), turn angle (theta) and ang. speed (w)}
        return self.drive_base.state()


class MobileModeling:
    def __init__(self, target_x=0.0, target_y=0.0):
        self.x = 0.0          # m
        self.y = 0.0          # m
        self.theta = 0.0      # rad
        self.last_distance_m = 0.0 # traveled distance
        self.x_target = target_x
        self.y_target = target_y

    def set_target(self, x_t: float, y_t: float):
        self.x_target = x_t
        self.y_target = y_t

    def update_pose(self, distance_mm: float, angle_deg: float):
        """Pure Dead Reckoning Pose Estimation from Wheel Encoders and Gyro"""
        distance_m = distance_mm / 1000.0  
        delta_s = distance_m - self.last_distance_m # step distance
        self.last_distance_m = distance_m

        self.theta = math.radians(angle_deg)
        # Override odometry angle with physical Hub IMU
        # self.theta = math.radians(self.robot.hub.imu.heading())
        

        # inttegrate incremental step
        self.x += delta_s * math.cos(self.theta)
        self.y += delta_s * math.sin(self.theta)

    def get_course_to_target(self):
        dx = self.x_target - self.x
        dy = self.y_target - self.y
        distance = math.sqrt(dx**2 + dy**2)
        
        alpha = math.atan2(dy, dx) - self.theta
        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi
        return distance, alpha

    def update_controls(self, kv=1.0, kw=2.0):
        """Lyapunov Direct Method controller."""
        d, alpha = self.get_course_to_target()

        # Scale kw smoothly down as d approaches zero to eliminate singularity chatter
        kw_adaptive = kw * (d / (d + 0.15))
        
        linear_v = kv * d * math.cos(alpha)
        angular_v = kv * math.sin(alpha) * math.cos(alpha) + kw * alpha
        # angular_v = kv * math.sin(alpha) * math.cos(alpha) + kw_adaptive * alpha
        return linear_v, angular_v

    def update_controls_bi(self, kv=1.0, kw=2.0):
        """Bi-Directional Lyapunov controls to eliminate J-Turns."""
        d, alpha = self.get_course_to_target()

        # If target is in the rear hemisphere, track with the back of the robot
        if abs(alpha) > math.pi / 2:
            alpha = alpha - math.copysign(math.pi, alpha)
            direction = -1.0
        else:
            direction = 1.0

        linear_v = direction * kv * d * math.cos(alpha)
        angular_v = kv * math.sin(alpha) * math.cos(alpha) + kw * alpha

        return linear_v, angular_v


                                                     
"""
Main
"""

def main():
    hub = InventorHub()
    robot = MobileRobot(hub)
    model = MobileModeling()
    robot.reset()
    target_ui = RemoteTargetUI(hub)
    distance_tolerance = 0.03  # 3 cm threshold

    try:
        while True:
            target_x, target_y = target_ui.get_target()
            model.set_target(target_x, target_y)
            print(f"\nDR Test Started: Target = ({target_x}m, {target_y}m)\n")
            timer = StopWatch()
            last_print_time = -500

            try:
                while True:
                    dist_mm, speed_mm_s, angle_deg, turn_rate_deg_s = robot.get_driving_state()
                    model.update_pose(dist_mm, angle_deg)
                    if target_ui.stop_requested():
                        print("\nTarget attempt cancelled (remote stop or disconnect).")
                        break

                    d_to_target, _ = model.get_course_to_target()
                    if d_to_target < distance_tolerance:
                        print(f"\nTarget Reached in {timer.time() / 1000.0:.2f} s")
                        break

                    v, w = model.update_controls(kv=1.2, kw=2.0)
                    robot.apply_controls(v, w)

                    current_time = timer.time()
                    if current_time - last_print_time >= 500:
                        print(
                            f"[{current_time / 1000.0:5.1f}s]" +
                            f" Dist to Target: {d_to_target:.3f}m |" +
                            f" Heading: {math.degrees(model.theta):6.1f}° |" +
                            f" v: {speed_mm_s / 1000.0:5.2f}m/s |" +
                            f" w: {turn_rate_deg_s:6.1f}°/s"
                        )
                        last_print_time = current_time
                    wait(40)
            finally:
                robot.stop()

            print(f"DR Final Estimated Pose: x={model.x:.3f}m, y={model.y:.3f}m, theta={math.degrees(model.theta):.1f}°")
    finally:
        robot.stop()

if __name__ == "__main__":
    main()