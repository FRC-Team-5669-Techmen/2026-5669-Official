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
| PhotonVision (photonlib) | USB camera AprilTag detection via PhotonCamera |
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
│   ├── GoobaToggleCommand      # Deploy/stow gooba (RETIRED: no hood motor)
│   ├── AutoGooba               # Auto-aim gooba using Limelight shot map (RETIRED)
│   ├── GooberAlign             # PID turret align w/ search sweep (Limelight)
│   ├── MariosEarCommand        # Multi-phase turret aim (Limelight → USB cams)
│   ├── ManualGoobaCommand      # D-pad jog gooba position up/down (RETIRED)
│   └── ManualTurretCommand     # D-pad jog turret left/right
├── subsystems/
│   ├── CommandSwerveDrivetrain # CTRE swerve + PathPlanner AutoBuilder + MapleSim
│   ├── ShooterSubsystem        # Dual TalonFX (leader/follower), velocity control
│   ├── IndexSubsystem          # Single TalonFX ("Spindex"), feeds the shooter
│   ├── ShooterIntakeSubsystem  # Single TalonFX, feeds fuel to shooter
│   ├── GoobaSubsystem          # Hood, Motion Magic position (RETIRED: no hood motor, not created)
│   ├── Goober                  # Turret motor, duty cycle rotation
│   ├── LimelightSubsystem      # Limelight NetworkTables (tx/ty/ta/tv/botpose)
│   ├── MariosEar               # Multi-camera vision: Limelight + 2 PhotonVision USB cams
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
- **Top roller**: Kraken X44, CAN 18 ("Shooter Top"). **Not coded yet**, so it does not spin.
- Velocity control (VelocityVoltage): fast 7000 RPM requested but clamped to `kMaxRPM` = 3100, slow 2500, idle 2000 (Driver D-pad Left), reverse -1000
- PID: kP=0.11, kV=0.12

### Index / Feeder
- Single TalonFX (CAN 13, Tuner X: "Spindex"), VoltageOut (FOC off, unlicensed): +12 V forward, -6 V reverse
- Feeds fuel from intake to shooter, brake mode

### Shooter Intake
- Single TalonFX (CAN 15, Tuner X: "UpTakeChannelForShooter"), DutyCycleOut (FOC off, unlicensed): forward constant 2.0 (saturates at 100%), reverse -0.6
- Pulls fuel into the shooter path, brake mode

### Gooba (Hood/Arc) (RETIRED, Oct 2026)
- **No hood motor on the robot right now.** Code kept but `GoobaSubsystem` is not created; hood bindings and named commands are no-ops.
- Notes below describe the old hood, for if it comes back:
- **Kraken X44** TalonFX (CAN 51)
- **Motion Magic** position control for smooth hood angle adjustment
- PID: kP=2.4, kD=0.1; cruise velocity 80 RPS, acceleration 160 RPS/s
- Positions: stowed=0.0, deployed=3.5 rotations (needs tuning)
- Has interpolating distance→rotation map (shot map) for auto-aim
- 40A supply current limit

### Goober (Turret)
- TalonFX (CAN 14, Tuner X: "AngleShooter"), duty cycle output (FOC off) with 1.0 speed multiplier
- Rotates turret to aim at targets, brake mode
- Firmware soft limits enabled: -25 to +10 rotations

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
- On CAN as ID 22 ("Intake motor(22)"), Pro-licensed. **Hold Driver Right Trigger** to spin it at `kRollerVolts` (VoltageOut, FOC off), coast mode, 40 A stator limit.
- Spins balls out instead of in? Flip `Constants.IntakeRoller.kInverted`. `kIntakeRollerEnabled = false` turns it off entirely (motor never created).

### Vision
- **Limelight**: AprilTag detection, provides tx/ty/ta/tv/botpose via NetworkTables
  - Mounted HORIZONTALLY and CENTERED on the shooter (2026 offseason remount): tx aims the turret, ty drives the distance trig
  - Distance-to-target calculation using trigonometry (`distanceToTarget()`)
- **PhotonVision USB cameras**: "Left" and "Right" cameras for wide-angle target search
- **MariosEar**: Fuses all 3 cameras into a multi-phase turret aiming pipeline

## Subsystem Responsibilities

| Subsystem | Hardware | Control Mode | Key Methods |
|-----------|----------|-------------|-------------|
| `CommandSwerveDrivetrain` | 8 TalonFX + 4 CANcoder + Pigeon2 | Field/Robot centric | `applyRequest()`, `seedFieldCentric()` |
| `ShooterSubsystem` | 2 TalonFX (leader/follower) | VelocityVoltage | `runAtRPM(rpm)`, `stop()` |
| `IndexSubsystem` | 1 TalonFX | VoltageOut (FOC off) | `run(volts)`, `stop()` |
| `ShooterIntakeSubsystem` | 1 TalonFX | DutyCycleOut (FOC off) | `run(speed)`, `stop()` |
| `GoobaSubsystem` (RETIRED, not created) | none | MotionMagicVoltage | `setPosition(rot)`, `getPosition()`, `getRotationForDistance(d)` |
| `Goober` | 1 TalonFX | DutyCycleOut | `setMotorSpeed(pct)`, `stop()` |
| `LimelightSubsystem` | Limelight camera | NetworkTables | `getTX()`, `getTY()`, `isValidTarget()`, `distanceToTarget()` |
| `MariosEar` | 2 PhotonVision + Limelight | PhotonCamera | `limelightHasTarget()`, `getLimelightTX()`, `getLeftResult()`, `getRightResult()` |
| `GroundIntakePivotSubsystem` | 2 TalonFX (20 leader, 21 follower) | MotionMagicVoltage + VoltageOut jog + Follower (FOC off) | `toggleCommand()`, `jogUpCommand()`, `jogDownCommand()`, `zeroCommand()`, `stopCommand()` |
| `ElevatorSubsystem` | 1 TalonFX (19) | VoltageOut (FOC off) + soft limits | `upCommand()`, `downCommand()`, `zeroCommand()`, `limitsOffCommand()`, `setVolts(v)`, `stop()` |
| `IntakeRollerSubsystem` | 1 TalonFX (22) | VoltageOut (FOC off) | `runCommand()` |

## Command Architecture

Commands follow the standard WPILib command-based pattern. Key patterns:

- **`FuelHandlingCommand`**: The core shooting/intake command. Coordinates index + shooterIntake + shooter simultaneously. `isForward=true` shoots, `isForward=false` reverses/intakes (-1000 RPM). Runs while held (autos add timeouts).
- **`FeedShooterCommand`**: Runs index + shooter intake only, so the shooter can be spun up independently (operator layout).
- **`MariosEarCommand`**: Multi-phase turret aiming with priority: (1) Limelight PID tracking → (2) Left USB cam coarse search → (3) Right USB cam coarse search → (4) stop.
- **`GooberAlign`**: Limelight PID turret alignment on raw tx (camera is centered on the shooter). When no valid target is in view it sweeps the turret between its soft limits hunting for one. Never finishes on its own — bind with whileTrue or a timeout.
- **`AutoGooba`** (RETIRED with the hood): Continuous auto-aim that maps Limelight distance (m) to hood position via the interpolating shot map, with a moving-average filter on distance.
- **Instant commands**: `GoobaToggleCommand` (retired) finishes immediately after setting state.
- **While-held commands**: `ManualTurretCommand`, the intake jog, elevator up/down and the roller (built with `startEnd`) run while held and stop on release.
- **Toggles**: Driver A flips the intake between UP and DOWN (`runOnce`). Operator RB turns `GooberAlign` on/off with `Trigger.toggleOnTrue`.

## Controller Bindings

### Driver Controller (Port 0 - Xbox)
This matches the drive team's official map. **LOCKED** = practiced, must stay exactly like this.

| Input | Action | Map |
|-------|--------|-----|
| Left Stick | Swerve translate (cubic scaling) | LOCKED |
| Right Stick | Swerve rotate | LOCKED |
| Left Bumper | Reset gyro / seed field centric | LOCKED |
| Right Trigger (hold) | Ground intake roller (Talon FX 22) | LOCKED |
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
| Right Bumper (press) | Toggle Limelight auto-aim (turret tracking on/off) | LOCKED |
| Y | Reserved, bound to nothing (was the "Gooba" hood) | LOCKED |
| D-Pad Up/Down (hold) | Jog ground intake up/down slowly | OPEN |
| D-Pad Left/Right (hold) | Manual turret jog | OPEN |
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
| `score` | GooberAlign → spin up → FuelHandling forward → stop | staged |
| `scorePreload` | Spin up → FuelHandling forward → stop (no vision) | staged |
| `visionScore` | GooberAlign → spin up → shoot → stop (hood step removed) | staged |
| `shoot` | FuelHandlingCommand (forward) | 2.0s |
| `stopShooter` | InstantCommand → shooter.stop() | - |
| `intake` | FuelHandlingCommand (reverse) | 2.0s |
| `aimTurret` | MariosEarCommand (multi-cam) | 2.0s |
| `alignTurret` | GooberAlign (Limelight-only) | 2.0s |
| `lowerIntake`, `togglePiston1`, `togglePiston2`, `runGroundIntake`, `deployGooba`, `stowGooba`, `autoAimGooba` | No-ops (pneumatics, intake roller and hood removed; kept so old autos load cleanly) | - |

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
| 18 | TalonFX (Kraken X44) | Shooter Top | **not coded yet** |
| 19 | TalonFX | Elevator | `Constants.Elevator` (climber) |
| 20 | TalonFX | Right Ground Intake Raise | `Constants.GroundIntakePivot.kLeaderId` |
| 21 | TalonFX | Left Ground Intake Raise | `Constants.GroundIntakePivot.kFollowerId` |
| 22 | TalonFX (Pro) | Intake motor(22) | `Constants.IntakeRoller` (ground intake roller) |
| 41 | Pigeon2 | Pigeon 2 | Swerve |

All devices are on the roboRIO's built-in CAN bus (`TunerConstants.kCANBus` is `""`), no CANivore.
IDs 51 and 61 no longer exist. The retired hood code still says 51 but is never created.
**Free IDs for new devices:** 23-40, 42-50, 52+. New CTRE devices ship as ID 0, so set the ID
in Phoenix Tuner X before writing code.

## Patterns & Conventions

- **Naming**: The team uses creative nicknames for subsystem classes: "Gooba" = hood/arc mechanism, "Goober" = turret, "MariosEar" = multi-camera vision system, "Marcos" = named-command registrar. Instance variables use plain descriptive names (`limelight`, `mariosEar`, `groundIntakePivot`, ...).
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

- **Shooter top roller (CAN 18) is not coded**: it won't spin when shooting. Needs a direction and an RPM
- **Elevator**: soft limits are measured and on (`kTopLimit` = 125.1, `kBottomLimit` = 0.0, Oct 24). Climbing the full robot may need more than 40 A stator; now that the limits are in, it's OK to raise `kStatorCurrentLimit` in small steps if a real climb stalls
- **Intake DOWN position**: `kDownPosition = -5.7`, measured Oct 23 (floor: right -5.73, left -5.83). Re-measure if the intake or its gearing changes
- **Intake roller**: confirm spin direction (`kInverted`) and speed (`kRollerVolts`) on the robot
- **Intake pivot sync**: Oct 23 Elastic data showed the two sides within ~0.03 rot while driven and ~0.1 rot at rest, so they're well coupled; the drift came from an unpowered free-fall after stopping early (fixed). Dual position control isn't needed unless that changes
- Verify the turret aim sign after the Limelight remount: if the turret runs away from the target, flip the sign in `GooberAlign` (`-limelight.getTX()`)
- Re-measure `Constants.Limelight.kMountAngleDegrees` and `kLensHeightMeters` on the new camera mount — `distanceToTarget()` is wrong until then
- `Constants.Shooter.kFastTargetRPM = 7000` is clamped to `kMaxRPM = 3100` at runtime — raise `kMaxRPM` when the flywheel is trusted at higher speeds
- X56 HOTAS is switched off; if re-enabled, its axis/button indices need on-hardware verification (docs/X56-HOTAS-SETUP.md)
- Servo mechanism(s) replacing the pneumatics: not built yet, not coded yet (use WPILib `Servo` on a roboRIO PWM port when ready)
- Hood retired: the shot map and AutoGooba are unused until a hood comes back
