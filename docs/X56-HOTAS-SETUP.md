# Logitech G X56 H.O.T.A.S. Setup

The robot supports the Logitech G X56 flight controller as an alternative driver
input. The Xbox controllers keep working exactly as before — when the X56 stick is
unplugged, every HOTAS binding is inert and the drive code falls back to the Xbox
controller automatically. No dashboard switch needed.

## Plugging it in

The X56 is **two separate USB devices** (stick and throttle). In the Driver Station
**USB Order** tab, drag them to these slots:

| Port | Device |
|------|--------|
| 0 | Driver Xbox controller |
| 1 | Operator Xbox controller |
| 2 | **X56 Stick** |
| 3 | **X56 Throttle** |

Ports are configured in `Constants.Hotas` (`kStickPort` / `kThrottlePort`).

> **The order matters.** If the X56 stick lands in slot 0 the code will read it as
> the driver Xbox controller. Check the USB tab every time before you enable: the
> list locks while the robot is enabled, and F1 re-scans it. An Xbox controller that
> ends up in slot 2 is ignored on purpose, so it can't take over the drive.

## Verify the axes and buttons FIRST

HID ordering can differ between firmware versions and OSes. Before driving:

1. Open the Driver Station **USB tab** and select the X56 stick.
2. Move each control and watch which axis/button lights up.
3. If anything disagrees with the table below, fix the index in `Constants.Hotas`.

Expected defaults:

| Control | Constant | Default index |
|---------|----------|---------------|
| Stick roll (left/right) | `kStickRollAxis` | axis 0 |
| Stick pitch (fwd/back) | `kStickPitchAxis` | axis 1 |
| Stick twist (yaw) | `kStickTwistAxis` | axis 2 |
| Trigger | `kTriggerButton` | button 1 |
| A (red thumb button) | `kButtonA` | button 2 |
| B | `kButtonB` | button 3 |
| C | `kButtonC` | button 4 |
| D | `kButtonD` | button 5 |
| Pinkie lever | `kPinkieButton` | button 6 |
| Right throttle lever | `kRightThrottleAxis` | throttle axis 1 |

If pulling the throttle back makes the robot *faster*, flip
`Constants.Hotas.kThrottleInverted`.

## Driving

| Control | Action |
|---------|--------|
| Stick pitch (push forward) | Drive downfield |
| Stick roll (tilt left/right) | Strafe |
| Stick twist | Rotate |
| **Right throttle lever** | Analog speed governor: full forward = 100%, pulled back = 15% crawl |

Same cubic response curve and slew limiting as the Xbox sticks. The Shuffleboard
Global Speed Limit still applies on top of the throttle scale.

## Button map (mirrors the driver Xbox layout)

| X56 control | Action | Xbox equivalent |
|-------------|--------|-----------------|
| Trigger, A | Unbound — ground intake retired | Right/Left Trigger |
| B, C | Unbound — free for future mechanisms (were the removed pneumatic pistons) | X, A |
| D | Toggle tank-drive mode | D-Pad Right |
| Pinkie lever | Re-zero field-centric heading | Left Bumper |
| Main hat up/down | Climb motor up/down (hold) | D-Pad Up/Down |

`SmartDashboard -> "HOTAS Connected"` shows whether the stick is detected.
`"HOTAS Throttle Connected"` shows the throttle; if the throttle is missing the robot
drives at full (global-limited) speed.

The X56 has no rumble motors. When you drive with it you won't feel the HUB-shift
rumble, so watch the `HUB OPEN` light on the dashboard (the Xbox driver controller
still rumbles if it's plugged in).

## First drive with the X56 (safety)

1. Put the robot **on blocks**, or set Shuffleboard **Global Speed Limit** to 25%.
2. Pull the throttle lever all the way back first. That's the 15% crawl setting.
3. Enable in teleop and touch nothing. The robot must sit still. If it creeps or
   spins, disable and check the axis numbers in the USB tab.
4. Push the stick forward. The robot should drive *away from you* (field-centric).
   Then twist and check the rotation direction. Then push the throttle lever forward
   and check that the speed goes up.

## Adding more bindings

`HotasX56` exposes `getStick()` / `getThrottle()` (both `CommandJoystick`) for raw
access to everything else — the throttle's SW1-6 switches, TGL toggles, rotaries,
mini sticks, etc. Find the button number in the DS USB tab, then bind in
`RobotContainer.configureHotasBindings()` and gate it with `.and(hotasConnected)`
like the existing bindings.
