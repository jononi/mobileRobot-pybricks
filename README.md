# mobileRobot-pybricks
Pybricks programmed autonomous mobile robot

## Remote target setup

Run the program on the Inventor Hub, then turn on a LEGO Powered Up remote
with its green button. The hub shows `C` while searching and connects to the
first available remote, retrying if necessary.

- Left `+` / `-`: change target X by 0.1 meters per press.
- Right `+` / `-`: change target Y by 0.1 meters per press.
- Left / right red center buttons: display X / Y without changing it.
- Green button: confirm the target; while driving, cancel the current attempt.

The 5x5 matrix shows the axis letter followed by its signed value in meters,
one character at a time. Wait for the display to finish before the next press.
The initial target is `(2.0, 0.0)` meters. Negative coordinates are supported.

After reaching a target or cancelling, the robot stops and returns to setup,
keeping the previous target for further editing. Targets are absolute coordinates
relative to the robot's original starting pose, not offsets from each stop.
Odometry is not reset between attempts.

A detected remote disconnection cancels the attempt and returns to connection
setup. Release and press green again to start another attempt. The hub's own
stop button retains its normal program-stop behavior.

## FSM navigation and obstacle avoidance

`main_fsm.py` is an alternative entry point; the original `main.py` is unchanged.
It imports `MobileRobot` and `MobileModeling` from `main.py`, so include both
files in your Pybricks project and run `main_fsm.py`. Motors remain on ports A
and B. Connect the forward-facing Powered Up ultrasonic sensor to port C
(or change `UltrasonicSensor(Port.C)` in the new script).

The FSM owns the complete operating cycle:

| State | Behavior | Display |
| --- | --- | --- |
| `IDLE` | Stopped; connect remote, edit or confirm an absolute target | `I` |
| `GO_TO_TARGET` | Forward-only target tracking, up to 0.15 m/s | `G` |
| `TURN_CLEAR` | Rotate right in place until forward clearance is detected | `T` |
| `BYPASS` | Hold the selected heading for a measured 0.4 m at 0.10 m/s | `B` |
| `BLOCKED` | Stopped after a sensor fault or navigation limit | `!` |

`C` indicates remote connection setup. Target editing uses the same buttons
and 0.1 m increments as the original script. Red buttons display X or Y;
coordinates appear one character at a time without blocking the control loop.
The initial target is `(2.0, 0.0)` m. To select `(5.0, 0.0)`, increase X with
the remote, or change the `IncrementalRemoteUI` constructor defaults.

Green confirms a target in `IDLE`, cancels motion, or acknowledges `BLOCKED`.
Acknowledgement returns to `IDLE`; release and press green again to retry.
Holding green does not repeatedly start or cancel attempts. Disconnection
cancels motion and returns to `IDLE`. Connection attempts may take up to one
second and only occur while stopped. Odometry continues across states and
attempts; the original absolute target is retained throughout each detour.

Readings below 300 mm trigger avoidance. Turning requires three consecutive
readings above 400 mm before entering `BYPASS`; a new obstruction during the
bypass starts another turn. After the bypass, target tracking resumes. Right
turns are preferred for the whole attempt; set `turn_sign=-1` for left turns.
The sign follows Pybricks' positive-clockwise heading convention, also used
by the existing pose estimator.

`NavigationFSM` constants configure speeds, thresholds and watchdogs. Defaults
stop with `BLOCKED` after a 90-degree or 8-second unsuccessful clearance search,
a 10-second unfinished bypass, 20 seconds without a new 5 cm improvement in
best target distance, 12 detours, or a 120-second overall attempt. These limits
are intentionally conservative and may reject otherwise navigable detours.
Remote cancellation has priority over navigation decisions. Sensor read errors
stop navigation; unexpected exceptions also stop the robot through `finally`.

This is reactive avoidance, not mapping or guaranteed path planning. A single
forward sensor cannot verify chassis side clearance or detect rear obstacles.
Its 2000 mm no-valid-echo value is treated as clear, but can also represent a
missed object. Test slowly with supervision and verify stopping distance and
turning clearance before trusting the 300 mm threshold. Concave obstacles,
wide obstacles and walls may require additional sensing or another algorithm.

Run hardware-free transition and remote UI tests on a computer with:

```sh
python -m unittest -v test_main_fsm
```

These tests do not validate actual sensor performance, motor stopping behavior
or hub firmware compatibility; those require testing on the robot.
