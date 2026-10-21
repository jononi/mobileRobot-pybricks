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
