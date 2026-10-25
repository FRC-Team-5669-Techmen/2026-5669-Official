# CLAUDE.md - FRC Team 5669 Techmen (2026 Season)

## Build & Deploy

```bash
# Build the project
./gradlew build

# Deploy to the RoboRIO
./gradlew deploy

# Run simulation
./gradlew simulateJava

# Run tests
./gradlew test
```

- **GradleRIO version**: 2026.2.1
- **Java**: 17
- **Team number**: loaded from `.wpilib/wpilib_preferences.json`
- Main class: `frc.robot.Main`

## Vendor Dependencies

| Library | Purpose |
|---------|---------|
| Phoenix6 (26.1.1) | CTRE TalonFX motors, CANcoders, Pigeon2 IMU |
| PathPlannerLib (2026.1.2) | Autonomous path following |
| PhotonVision (photonlib) | USB camera AprilTag detection via PhotonCamera (unused: no USB cameras on the robot) |
| DogLog | Telemetry logging (NT publish, DS capture, PDH logging) |
| MapleSim (0.4.0-beta) | Physics simulation for swerve drivetrain |
| WPILib NewCommands | Command-based framework |

## Project Structure

```
src/main/java/frc/robot/
├── Main.java                  # Entry point
├── Robot.java                 # TimedRobot lifecycle, rumble feedback, DogLog init
├── RobotContainer.java        # Subsystem instantiation, bindings, auto chooser
├── Constants.java             # All CAN IDs, PID gains, speeds, positions
├── Marcos.java                # PathPlanner named command registration
├── HubShiftTracker.java       # REBUILT 2026 hub active/inactive shift logic
├── DriverDashboard.java       # Shuffleboard "Match Dash" tab (match phase, hub status, RPM)
├── Telemetry.java             # Swerve state → NetworkTables + SignalLogger
├── LimelightHelpers.java      # Limelight vendor helper (not directly used)
├── controls/
│   └── HotasX56               # Logitech X56 HOTAS wrapper (stick + throttle USB devices)
├── commands/
│   ├── FuelHandlingCommand     # Coordinated shooter + index + intake (fwd/rev)
│   ├── FeedShooterCommand      # Index + shooter intake only (shooter runs separately)
│   ├── RunShooterCommand       # Spin shooter at target RPM
│   ├── GoobaToggleCommand      # Deploy/stow the hood (named commands deployGooba/stowGooba)
│   ├── AutoGooba               # Hood follows the Limelight ty through the shot map (runs with auto-aim)
│   ├── GooberAlign             # PID turret align w/ search sweep (Limelight)
│   ├── MariosEarCommand        # Multi-phase turret aim (Limelight → USB cams) (RETIRED: no USB cams)
│   ├── ManualGoobaCommand      # Operator Left Stick jog of the hood up/down
│   └── ManualTurretCommand     # D-pad jog turret left/right
├── subsystems/
│   ├── CommandSwerveDrivetrain # CTRE swerve + PathPlanner AutoBuilder + MapleSim
│   ├── ShooterSubsystem        # Dual TalonFX (leader/follower), velocity control
│   ├── IndexSubsystem          # Single TalonFX ("Spindex"), feeds the shooter
│   ├── ShooterIntakeSubsystem  # Single TalonFX, feeds fuel to shooter
│   ├── GoobaSubsystem          # Hood, TalonFX 18, Motion Magic position + shot map
│   ├── Goober                  # Turret motor, duty cycle rotation, ~270° from HOME with soft limits
│   ├── LimelightSubsystem      # Limelight NetworkTables (tx/ty/ta/tv/botpose)
│   ├── MariosEar               # Multi-camera vision: Limelight + 2 PhotonVision USB cams (RETIRED, not created)
│   ├── GroundIntakePivotSubsystem # 2 TalonFX (20 leader, 21 follower): Motion Magic UP/DOWN toggle + jog
│   ├── ElevatorSubsystem       # Climber, TalonFX 19: hold-to-move up/down
│   └── IntakeRollerSubsystem   # Ground intake roller, TalonFX 22: hold Driver RT to spin
├── generated/
│   └── TunerConstants.java     # Tuner X generated swerve config (module IDs, offsets, gains)
└── sim/
    └── MapleSimSwerveDrivetrain # MapleSim physics simulation wrapper
```

## Robot Mechanisms

This is a 2026 FRC robot with the following mechanical systems:

### Swerve Drivetrain
- 4-module swerve drive using **Kraken X60** motors (TalonFX) + CANcoders
- Pigeon2 IMU (CAN ID 41) for heading
- Max speed: **4.39 m/s** at 12V
- Drive gear ratio: 7.03125, steer gear ratio: 26.09
- Wheel radius: 2 inches
- Robot footprint: ~29.5" x 24.5" (bumpers)
- Supports field-centric and robot-centric drive modes (Shuffleboard selectable)

### Shooter System
- **Bottom roller**: TalonFX CAN 16 + CAN 17 (both named "Shooter Bottom" in Tuner X), mounted mirrored.
  The code drives both at the same speed with opposite inversion, which is correct for a mirrored pair.
- **Top roller**: belted to the bottom roller, so it spins whenever the bottom roller does. **Nothing to code.** The Kraken X44 on CAN 18 (Tuner X name "Shooter Top") drives the **hood**, not this roller (programming lead, Oct 25).
- Velocity control (VelocityVoltage): fast 7000 RPM requested but clamped to `kMaxRPM` = 3100, slow 2500, idle 2000 (Driver D-pad Left), reverse -1000
- PID: kP=0.11, kV=0.12

### Index / Feeder
- Single TalonFX (CAN 13, Tuner X: "Spindex"), VoltageOut (FOC off, unlicensed): +12 V forward, -6 V reverse
- Feeds fuel from intake to shooter, brake mode

### Shooter Intake
- Single TalonFX (CAN 15, Tuner X: "UpTakeChannelForShooter"), DutyCycleOut (FOC off, unlicensed): forward constant 2.0 (saturates at 100%), reverse -0.6
- Pulls fuel into the shooter path, brake mode

### Gooba (Hood)
- **Kraken X44 on Talon FX 18** (Tuner X name "Shooter Top", no Pro license, so FOC off). Tilts the hood up/down to change the shot. Same code as last season, which ran it on ID 51. `Constants.Gooba.kHoodEnabled = false` turns it off entirely (motor never created).
- **Motion Magic** position control: kP=2.4, kD=0.1, cruise 80 rot/s, accel 160 rot/s². Brake, 40 A supply + 30 A stator limits.
- **0 = power-on position, so always power on with the hood all the way DOWN.** No re-zero at startup (last season's code did). Dashboard button **"Zero Hood (hood is DOWN)"**. **Positive = UP** (flip `kInverted` if the jog Up lowers it).
- Soft limits **0 to `kTopPosition` = 10.7 rotations** (last season's top). Lower it if the hood hits the top first.
- **Shot map** keyed on the Limelight's **ty** (up/down angle to the tag): last season's competition points 27.3→1.0, -16.8→2.36, -18.3→3.8, -23.9→5.1. They were recorded from raw tx on the old sideways mount (tx was the up/down angle then); the camera is in the same spot, so they carry over as ty. Close to the hub = big positive ty.
- **Controls**: Operator **RB** auto-aim now moves the hood too (`AutoGooba`, holds its spot when no tag). Operator **Y** toggles stowed (0.0) / deployed (3.5). Operator **Left Stick up/down** (hold) jogs it (2 rot/s). Any of these manual controls cancels auto-aim.
- **When the robot disables**, the hood target is dropped, so it never jumps on enable.
- Dashboard: "Hood Enabled", "Hood Position (rot)", "Hood Target (rot)", "Hood Amps", "AutoGooba ty", "AutoGooba Target Rot".

### Goober (Turret)
- TalonFX (CAN 14, Tuner X: "AngleShooter"), duty cycle output (FOC off), brake mode, 30 A stator limit (low on purpose, to go easy on the wires).
- Turns the shooter **and the Limelight** (the camera rides on the turret). The Pigeon is on the frame, so turning the turret doesn't affect the gyro.
- **Range: about 270°, not 360°.** The motor and Limelight wires run up to the turret, and they are the only thing that stops it. **HOME** = one END of the travel, lined up with the spot mark. From HOME it can only turn **RIGHT** (clockwise seen from above) about 270°, then back.
- **0 = power-on position, so always power on with the turret on the HOME mark.** The code does **not** re-zero at startup. Dashboard button **"Zero Turret (turret at HOME)"** (works while disabled). If it was powered on somewhere else: power off, turn it by hand to the mark, power on.
- **Positive = turning RIGHT = away from HOME.** If Operator D-pad Right turns it LEFT, flip `kMotorInverted`.
- **Soft limits**: HOME side at 0 (always on). Far side (270° from HOME) at the saved far end, on once it's set. **Set it from the dashboard**: power on at HOME, hold Operator D-pad Right until it's just short of where the wires get tight, press **"Set Turret Far End (turret at far end)"** (works while disabled; refuses if the turret is near HOME). It's saved on the roboRIO (WPILib Preferences, key "Turret End Position (rot)"), so it survives power-offs and deploys, and shows as "Turret Far End (rot)". **"Clear Turret Far End"** forgets it, to re-measure. `Constants.Turret.kEndPosition` is the backup used when nothing is saved: **36** (team decision Oct 24; the far end first read 32.49). **If neither is set**: NOTHING stops the turret past 270° ("Turret Status" warns), and auto-aim does nothing. `kEndDegrees` (270) turns rotations into degrees.
- Auto-aim and the search sweep stay `kAimMarginDegrees` (10°) inside both ends. If the tag is further around than that, auto-aim stops and says TAG OUT OF REACH.
- Dashboard: "Turret Status" (setup / At HOME end / At FAR end / OK), "Turret Auto-Aim" (OFF / SEARCHING / TRACKING / LOCKED / TAG OUT OF REACH), "Turret Position (rot)", "Turret Angle (deg)" (0 = HOME), "Turret Far End (rot)", "Turret Amps".

### Ground Intake Pivot (raise/lower)
- **Leader**: TalonFX CAN 20 ("Right Ground Intake Raise"). The code only ever commands this one.
- **Follower**: TalonFX CAN 21 ("Left Ground Intake Raise"), set once at startup to copy the leader with `Follower(20, Opposed)` (mirrored mount). No soft limits on the follower.
- **Driver A toggles UP/DOWN** (FOC off). Pure toggle: it remembers the last press and alternates. When the code starts it checks the position once: below halfway counts as DOWN, so after a redeploy with the intake down the first press raises it (at power-on the motors read 0, so the first press goes DOWN). The A command is instant and schedules a fresh move, so every press counts even mid-move. Dashboard "Intake Next A Press" shows what the next press will do.
  - **DOWN**: Motion Magic down; once it has **arrived (close and stopped moving) or stalled on the floor**, it stops pushing and rests in brake (no humming). Waiting for arrival keeps both sides under motor control the whole way; stopping early let the intake free-fall and the sides drifted apart.
  - **UP**: Motion Magic up to `kUpHoldOffset` (0.5 rot) **below** UP = 0 and **keeps holding** there until the next press, so it can't drop mid-match (faint hum is normal). It never touches the top hard stop. Bigger offset = stops lower.
  - "Stall" = more than `kStallAmps` while slower than `kStallSpeed` for `kStallSeconds` (dashboard "Intake Stalled").
  - **ZERO WRONG**: if DOWN stops more than `kZeroWrongTolerance` short of the floor, "Intake Status" says `ZERO WRONG by X: push intake up, press Zero Intake`. Cleared by Zero Intake or the next clean DOWN.
- **Positive = UP.** No sensor knows the real angle (no shaft for a CANcoder; the mentor's rule is to always start with the intake up). UP = 0 comes from the Talon FX starting at 0 when the robot is **powered on**, so **push the intake all the way up against the top before every power-on**. It sags with the power off, and even a sag of a fraction of a rotation shifts every position: Oct 25 it powered on 0.9 low, DOWN slammed the floor and UP stopped too low to stay up (that was with the old let-go-at-UP code). The code does **not** re-zero at startup, so deploying new code keeps the true position. If the zero is ever wrong: push the intake up by hand and press Zero Intake. DOWN (`kDownPosition`) is a measured **negative** number. While it's 0.0, A does nothing and the dashboard says "Intake DOWN position not set". A stops `kDownBackoff` short of DOWN so the motors don't stall into the ground.
- **Operator D-pad Up/Down** jogs it slowly (`kJogVolts`, VoltageOut, FOC off). The jog is a manual override: it **ignores the soft limits** while held (they come back on release), so it can always recover the intake if the zero is off.
- **Soft limit on the leader: DOWN only** (`kSoftLimitMargin` past DOWN, once DOWN is set). No UP soft limit: an UP limit measured from a wrong zero is what used to block raising.
- Brake mode, 40 A stator limit on both motors. Speed is set by `kCruiseVelocity` / `kAcceleration` / `kJerk`; holding by `kP` / `kD` / `kV`.
- **Enabling never moves it**: when the robot disables, the Motion Magic target is cleared, so it sits (brake) until the next A press or jog.
- Dashboard: "Intake Status", "Intake Position (rot)", "Intake Target (rot)", "Intake Holding Target", both motors' amps and speeds, plus a **"Zero Intake (intake is UP)"** button (works while disabled).
- Wrong direction: flip `kLeaderInverted`. Motors fighting: flip `kFollowerAlignment`. All in `Constants.GroundIntakePivot`.

### Elevator / Climber (Talon FX 19)
- Rises, latches on, pulls the robot up. **Hold Driver D-pad Up/Down** to move, release to stop. Positive = UP.
- `VoltageOut` (FOC off) at `kElevatorVolts` (2.0 V after a good first test at 1.5 V); direction via `kElevatorInverted`. Brake mode, 40 A stator limit.
- **Soft limits** (`kTopLimit` / `kBottomLimit`): the motor stops by itself at the top and bottom, even with the D-pad held, so it never stalls into the ends. **Measured Oct 24: `kTopLimit` = 125.1, `kBottomLimit` = 0.0, so the limits are ON** (the code turns them on whenever top > bottom). 0.0 is the power-on spot (fully down), so D-pad Down brings it all the way back to where it started; if it ever bangs or stalls at the bottom, use a small positive number instead (like 0.5). Dashboard "Elevator Status" shows OK / At TOP limit / At BOTTOM limit. Re-measure from "Elevator Position (rot)" if the elevator's gearing changes.
- **0 = power-on position, so power on with the elevator fully DOWN.** The code does **not** re-zero at startup (a redeploy with the elevator up would put the limits in the wrong place). Dashboard buttons: **"Zero Elevator (elevator is DOWN)"** and **"Elevator Limits OFF (recovery)"** (click to turn limits off, click again to turn them back on; for when it was powered on with the elevator up and a limit blocks the D-pad). No stall detection on purpose: during a climb the motor is slow and working hard, which looks like a stall, and cutting power there would drop the robot.
- The old climb code (motor 21) was deleted; the elevator replaced it.

### Ground Intake Roller (Talon FX 22)
- On CAN as ID 22 ("Intake motor(22)"), Pro-licensed. **Hold Driver Right Trigger** to spin it in at `kRollerVolts` (**9 V**, was 6), **hold Driver Left Trigger** to spin it backwards at `kReverseVolts` (-9 V) and spit balls out. VoltageOut (FOC off), coast mode, 40 A stator limit.
- Spins balls out instead of in? Flip `Constants.IntakeRoller.kInverted`. `kIntakeRollerEnabled = false` turns it off entirely (motor never created).

### Vision
- **Limelight**: the robot's **only** camera. AprilTag detection, provides tx/ty/ta/tv/tid/botpose via NetworkTables (table name `limelight`). Model not confirmed (newer black finned body, no green LED ring; likely a 3G or 4): the web UI at `http://limelight.local:5801` shows it.
  - Mounted HORIZONTALLY and CENTERED on the shooter, **on the turret**, so it turns with it: tx aims the turret (tx > 0 = tag to the right), ty drives the distance trig
  - **Hub tags only, and only OURS**: the turret aims at the center tag of each of the 3 hub faces we shoot at. Red hub = 2, 5, 10; blue hub = 18, 21, 26 (checked against the WPILib 2026 field layout). Until the Driver Station reports the alliance it accepts both; after that only ours (dashboard "Limelight Aiming At"). For practice, set the team station (Red/Blue) in the Driver Station.
  - Dashboard: "Limelight tx", "Limelight Tag ID", "Limelight Aiming At", "Limelight distance (m)"
  - Distance-to-target calculation using trigonometry (`distanceToTarget()`)
- **PhotonVision USB cameras / MariosEar (RETIRED, Oct 2026)**: there are no USB cameras on the robot. `MariosEar` and `MariosEarCommand` are kept but never created.

## Subsystem Responsibilities

| Subsystem | Hardware | Control Mode | Key Methods |
|-----------|----------|-------------|-------------|
| `CommandSwerveDrivetrain` | 8 TalonFX + 4 CANcoder + Pigeon2 | Field/Robot centric | `applyRequest()`, `seedFieldCentric()` |
| `ShooterSubsystem` | 2 TalonFX (leader/follower) | VelocityVoltage | `runAtRPM(rpm)`, `stop()` |
| `IndexSubsystem` | 1 TalonFX | VoltageOut (FOC off) | `run(volts)`, `stop()` |
| `ShooterIntakeSubsystem` | 1 TalonFX | DutyCycleOut (FOC off) | `run(speed)`, `stop()` |
| `GoobaSubsystem` (hood) | 1 TalonFX (18) | MotionMagicVoltage (FOC off) + soft limits | `setPosition(rot)`, `getPosition()`, `getRotationForTy(ty)`, `toggleCommand()`, `zeroCommand()` |
| `Goober` | 1 TalonFX (14) | DutyCycleOut (FOC off) + soft limits | `setMotorSpeed(pct)`, `aimAtTarget(tx)`, `zeroCommand()`, `stop()` |
| `LimelightSubsystem` | Limelight camera | NetworkTables | `getTX()`, `getTY()`, `isValidTarget()`, `distanceToTarget()` |
| `MariosEar` (RETIRED, not created) | none (no USB cameras) | PhotonCamera | `limelightHasTarget()`, `getLimelightTX()`, `getLeftResult()`, `getRightResult()` |
| `GroundIntakePivotSubsystem` | 2 TalonFX (20 leader, 21 follower) | MotionMagicVoltage + VoltageOut jog + Follower (FOC off) | `toggleCommand()`, `jogUpCommand()`, `jogDownCommand()`, `zeroCommand()`, `stopCommand()` |
| `ElevatorSubsystem` | 1 TalonFX (19) | VoltageOut (FOC off) + soft limits | `upCommand()`, `downCommand()`, `zeroCommand()`, `limitsOffCommand()`, `setVolts(v)`, `stop()` |
| `IntakeRollerSubsystem` | 1 TalonFX (22) | VoltageOut (FOC off) | `runCommand()`, `reverseCommand()` |

## Command Architecture

Commands follow the standard WPILib command-based pattern. Key patterns:

- **`FuelHandlingCommand`**: The core shooting/intake command. Coordinates index + shooterIntake + shooter simultaneously. `isForward=true` shoots, `isForward=false` reverses/intakes (-1000 RPM). Runs while held (autos add timeouts).
- **`FeedShooterCommand`**: Runs index + shooter intake only, so the shooter can be spun up independently (operator layout).
- **`MariosEarCommand`** (RETIRED, no USB cameras): Multi-phase turret aiming with priority: (1) Limelight PID tracking → (2) Left USB cam coarse search → (3) Right USB cam coarse search → (4) stop. Don't revive it as is: it swings at 100% power and ignores the turret's range.
- **`GooberAlign`**: Limelight PID turret alignment on raw tx (camera is centered on the shooter): tx > 0 (tag on the right) turns the turret right. When no valid target is in view it sweeps the turret between just inside HOME and just inside the far end hunting for one. Does nothing until the far end is set. Never finishes on its own — bind with whileTrue or a timeout.
- **`AutoGooba`**: Continuous hood auto-aim that maps Limelight ty to hood position via the interpolating shot map, with a moving-average filter on ty. Runs alongside `GooberAlign` (RB toggle, `visionScore`, `score`). Requires only the hood.
- **Instant commands**: `GoobaToggleCommand` finishes immediately after setting the hood target.
- **While-held commands**: `ManualTurretCommand`, the intake jog, elevator up/down and the roller (built with `startEnd`) run while held and stop on release.
- **Toggles**: Driver A flips the intake between UP and DOWN (`runOnce`). Operator RB turns `GooberAlign` + `AutoGooba` on/off with `Trigger.toggleOnTrue`. Operator Y flips the hood stowed/deployed.

## Controller Bindings

### Driver Controller (Port 0 - Xbox)
This matches the drive team's official map. **LOCKED** = practiced, must stay exactly like this.

| Input | Action | Map |
|-------|--------|-----|
| Left Stick | Swerve translate (cubic scaling) | LOCKED |
| Right Stick | Swerve rotate | LOCKED |
| Left Bumper | Reset gyro / seed field centric | LOCKED |
| Right Trigger (hold) | Ground intake roller (Talon FX 22) | LOCKED |
| Left Trigger (hold) | Ground intake roller **backwards** (spit balls out) | added Oct 25 |
| A (press) | Toggle ground intake DOWN / UP (does nothing until `kDownPosition` is set) | LOCKED |
| X | Reserved, bound to nothing (was the climb piston) | LOCKED |
| D-Pad Up (hold) | Climb up (elevator 19) | LOCKED |
| D-Pad Down (hold) | Climb down (elevator 19) | LOCKED |
| D-Pad Left (press) | Toggle shooter idle at 2000 RPM (press again = off) | LOCKED |
| Start (hold) | Slow mode (button speed limiter) | LOCKED |

### Operator Controller (Port 1 - Xbox)
| Input | Action | Map |
|-------|--------|-----|
| Right Trigger (hold) | Shoot (Spindex + uptake) | LOCKED |
| Left Trigger (hold) | Shooter windup | LOCKED |
| Right Bumper (press) | Toggle Limelight auto-aim (turret tracking + hood on/off) | LOCKED |
| Y (press) | Toggle hood stowed / deployed ("Toggle Gooba Hood", as on the team map) | LOCKED |
| Left Stick up/down (hold) | Jog the hood up/down slowly | OPEN |
| D-Pad Up/Down (hold) | Jog ground intake up/down slowly | OPEN |
| D-Pad Left/Right (hold) | Manual turret jog (Right = turn right, away from HOME; diagonals count, so it doesn't stutter) | OPEN |
| B (hold) | Spindex backwards to clear a jam | not on map, kept on purpose |

Removed because they weren't on the map: Driver D-pad Right (tank-drive toggle, and tank mode itself) and Operator Left Bumper (aim while held).

### HOTAS (Ports 2/3 - Logitech X56): SWITCHED OFF
`Constants.Hotas.kEnabled = false`, so only the two Xbox controllers can drive. The code stays in the
project for practice (set the flag true). Even when on, it can **never** use ports 0 or 1, and it
ignores an Xbox controller that lands in its slot. See `docs/X56-HOTAS-SETUP.md`.

### Speed Limiting
- **Global speed limiter**: Shuffleboard chooser (100%/75%/50%/25%), always active
- **Button speed limiter**: Shuffleboard chooser (50%/75%/25%/10%), active only while Start is held (Xbox only)
- **HOTAS throttle** (only when HOTAS is switched on): right throttle lever scales speed 15%-100% on top of the global limiter

### Rumble Feedback (driver controller, TELEOP only)
- Constant low rumble (0.1) while our HUB is open for scoring (`HubShiftTracker`)
- Pulsing rumble (0.9) during the last 5 seconds of each 25s SHIFT window

## PathPlanner Setup

### Configuration
- Holonomic mode, robot size 0.9m x 0.9m
- Default max velocity: 3.0 m/s, max acceleration: 3.0 m/s^2
- Default max angular velocity: 540 deg/s, max angular acceleration: 720 deg/s^2
- Robot mass: 74.088 kg, MOI: 6.883
- Drive motor: Kraken X60, wheel COF: 1.1

### AutoBuilder
- Configured in `CommandSwerveDrivetrain.configureAutoBuilder()`
- Translation PID: (10, 0, 0), Rotation PID: (7, 0, 0)
- Alliance-aware path flipping (Red vs Blue)
- Auto chooser defaults to "Tests"

### Named Commands (registered in `Marcos.java`)
The registered strings are referenced by the deployed `.auto`/`.path` files — do not
rename them without updating the autos in `src/main/deploy/pathplanner/`.

| Name | Command | Timeout |
|------|---------|---------|
| `spinUpShooter` | RunShooterCommand (fast RPM) | 1.5s |
| `spinUpShooterSlow` | RunShooterCommand (slow RPM) | 1.5s |
| `score` | GooberAlign + AutoGooba → spin up → FuelHandling forward → stop | staged |
| `scorePreload` | Spin up → FuelHandling forward → stop (no vision) | staged |
| `visionScore` | GooberAlign + AutoGooba (2.5 s) → spin up → shoot → stop | staged |
| `shoot` | FuelHandlingCommand (forward) | 2.0s |
| `stopShooter` | InstantCommand → shooter.stop() | - |
| `intake` | FuelHandlingCommand (reverse) | 2.0s |
| `aimTurret` | GooberAlign (Limelight-only; was MariosEarCommand, no USB cameras now) | 2.0s |
| `alignTurret` | GooberAlign (Limelight-only) | 2.0s |
| `deployGooba` / `stowGooba` | Hood to deployed / stowed preset | instant |
| `autoAimGooba` | AutoGooba (hood follows ty) | 2.0s |
| `lowerIntake`, `togglePiston1`, `togglePiston2`, `runGroundIntake` | No-ops (pneumatics removed; kept so old autos load cleanly) | - |

### Paths & Autos
- **Autos**: `Center Preload - Dumb`, `Center Preload - Vision`, `Left 3-Piece Auto`, `Right 3-Piece Auto`
- **Paths**: see `src/main/deploy/pathplanner/paths/`

## CAN ID Map

Source of truth: **Phoenix Tuner X** (re-checked Oct 2026, all Talon FX on firmware 26.3.0.0).
For mechanisms, "Name" is exactly what the device is called in Tuner X, so you can match code to hardware.
For swerve it is the module position.

| ID | Device | Name | Code |
|----|--------|--------------|------|
| 1 | TalonFX | Back Right turn | Swerve (`TunerConstants`) |
| 2 | TalonFX | Front Right turn | Swerve |
| 3 | TalonFX | Back Left drive | Swerve |
| 4 | TalonFX | Back Right drive | Swerve |
| 5 | TalonFX | Front Left turn | Swerve |
| 6 | TalonFX | Front Left drive | Swerve |
| 7 | TalonFX | Back Left turn | Swerve |
| 8 | TalonFX | Front Right drive | Swerve |
| 9 | CANcoder | Front Right | Swerve |
| 10 | CANcoder | Front Left | Swerve |
| 11 | CANcoder | Back Left | Swerve |
| 12 | CANcoder | Back Right | Swerve |
| 13 | TalonFX | Spindex | `Constants.Index` |
| 14 | TalonFX | AngleShooter | `Constants.Turret` (Goober) |
| 15 | TalonFX | UpTakeChannelForShooter | `Constants.ShooterIntake` |
| 16 | TalonFX | Shooter Bottom | `Constants.Shooter.kLeaderId` |
| 17 | TalonFX | Shooter Bottom | `Constants.Shooter.kFollowerId` |
| 18 | TalonFX (Kraken X44) | Shooter Top | `Constants.Gooba` (**hood**; the name is old) |
| 19 | TalonFX | Elevator | `Constants.Elevator` (climber) |
| 20 | TalonFX | Right Ground Intake Raise | `Constants.GroundIntakePivot.kLeaderId` |
| 21 | TalonFX | Left Ground Intake Raise | `Constants.GroundIntakePivot.kFollowerId` |
| 22 | TalonFX (Pro) | Intake motor(22) | `Constants.IntakeRoller` (ground intake roller) |
| 41 | Pigeon2 | Pigeon 2 | Swerve |

All devices are on the roboRIO's built-in CAN bus (`TunerConstants.kCANBus` is `""`), no CANivore.
IDs 51 and 61 no longer exist (the hood moved from 51 to 18).
**Free IDs for new devices:** 23-40, 42-50, 52+. New CTRE devices ship as ID 0, so set the ID
in Phoenix Tuner X before writing code.

## Patterns & Conventions

- **Naming**: The team uses creative nicknames for subsystem classes: "Gooba" = hood, "Goober" = turret (easy to mix up with Gooba), "MariosEar" = multi-camera vision system (retired), "Marcos" = named-command registrar. Instance variables use plain descriptive names (`limelight`, `goober`, `groundIntakePivot`, ...).
- **Vision mounting**: The Limelight is mounted HORIZONTALLY and CENTERED on the shooter (2026 offseason remount) — `tx` aims the turret, `ty` feeds the distance trig. Re-measure `Constants.Limelight.kMountAngleDegrees`/`kLensHeightMeters` whenever the mount changes.
- **Hub shifts**: `HubShiftTracker.isOurHubOpen()` implements the REBUILT shift schedule (game data at teleop start + match timer). Driver rumble and the dashboards read it via `RobotContainer.isHubOpenForUs()`.
- **Constants**: All hardware IDs, PID gains, speeds, and positions live in `Constants.java` inner classes. Comments mark values as `//fixed` when confirmed on hardware.
- **Marcos.java**: Centralized PathPlanner named command registration, separate from RobotContainer. Named commands must be registered before `AutoBuilder.buildAutoChooser()`.
- **Motor control**: Phoenix 6 API throughout. TalonFX motors use `DutyCycleOut` for simple speed control, `VelocityVoltage` for RPM control (shooter), `MotionMagicVoltage` for smooth position control (retired hood), and `VoltageOut` + `Follower` for the two-motor intake pivot. A follower is set once at startup and must never be sent any other command.
- **Phoenix Pro licensing / FOC**: **only Talon FX 16, 17 (shooter bottom) and 22 (intake roller) have Phoenix Pro.** Phoenix 6 control requests turn FOC on by default, and FOC is a paid Pro feature: an unlicensed Talon FX refuses FOC commands and won't move (LED alternates red/green). **Every control request for any other motor must use `.withEnableFOC(false)`**. All current code does.
- **Simulation**: MapleSim-backed swerve physics simulation with 0.002s loop period. Simulated robot: 115 lbs, KrakenX60 motors, 1.2 tire COF.
- **Logging**: DogLog for structured logging (battery, odometry, module states). Telemetry class publishes swerve state to NetworkTables + SignalLogger. HootAutoReplay for timestamp/joystick replay.
- **Drive input**: Cubic scaling on joystick axes (`Math.pow(abs, 3) * signum`) for fine control at low speeds.
- **TunerConstants**: Auto-generated by CTRE Tuner X, contains all swerve module configurations. Do not manually edit without re-running Tuner X.

## Known TODOs / FIXMEs

- **Elevator**: soft limits are measured and on (`kTopLimit` = 125.1, `kBottomLimit` = 0.0, Oct 24). Climbing the full robot may need more than 40 A stator; now that the limits are in, it's OK to raise `kStatorCurrentLimit` in small steps if a real climb stalls
- **Intake DOWN position**: `kDownPosition = -5.7`, measured Oct 23 (floor: right -5.73, left -5.83). Re-measure if the intake or its gearing changes
- **Intake roller**: confirm spin direction (`kInverted`) and speed (`kRollerVolts`) on the robot
- **Intake pivot sync**: Oct 23 Elastic data showed the two sides within ~0.03 rot while driven and ~0.1 rot at rest, so they're well coupled; the drift came from an unpowered free-fall after stopping early (fixed). Dual position control isn't needed unless that changes
- **Turret setup**: far end set to 36 rotations (`kEndPosition`). If 36 rotations is more than 270°, raise `kEndDegrees` to match so "Turret Angle (deg)" and the 10° margin stay right. Still to confirm D-pad Right turns it right (else flip `kMotorInverted`). Then check the aim: a tag held to the right of the camera must read a positive "Limelight tx" (if negative, the image is flipped in the Limelight settings), and auto-aim must turn toward the tag
- **Limelight pose into odometry is wrong with the camera on the turret**: `CommandSwerveDrivetrain.periodic()` feeds MegaTag2 poses into odometry, but MegaTag2 assumes the camera is fixed to the frame. Any turret angle other than the one set in the Limelight UI gives a wrong position (autos can drive to the wrong place). It also passes the raw Pigeon yaw instead of the drivetrain heading. Swerve code, so not changed yet: waiting on the programming lead's OK
- Re-measure `Constants.Limelight.kMountAngleDegrees` and `kLensHeightMeters` on the new camera mount — `distanceToTarget()` is wrong until then
- `Constants.Shooter.kFastTargetRPM = 7000` is clamped to `kMaxRPM = 3100` at runtime — raise `kMaxRPM` when the flywheel is trusted at higher speeds
- X56 HOTAS is switched off; if re-enabled, its axis/button indices need on-hardware verification (docs/X56-HOTAS-SETUP.md)
- Servo mechanism(s) replacing the pneumatics: not built yet, not coded yet (use WPILib `Servo` on a roboRIO PWM port when ready)
- **Hood bring-up (Oct 25)**: confirm Left Stick Up raises it (else flip `Gooba.kInverted`), that 10.7 rotations is the real top, and that last season's ty shot map still scores; re-record points if not
- **Limelight localization** (field position from AprilTags, so the turret aims at the hub center and the hood uses true distance): needs the turret-camera fix in `CommandSwerveDrivetrain`, plus measurements (turret center offset, camera offset/height/tilt, HOME direction, exact degrees per turret rotation)
