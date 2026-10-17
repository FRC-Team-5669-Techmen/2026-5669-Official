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
│   └── GroundIntakePivotSubsystem # 2 TalonFX (20 leader, 21 follower): raise/lower the intake
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
- Velocity control (VelocityVoltage): fast 7000 RPM requested but clamped to `kMaxRPM` = 3100, slow 2500, idle 1700, reverse -1000
- PID: kP=0.11, kV=0.12

### Index / Feeder
- Single TalonFX (CAN 13, Tuner X: "Spindex"), VoltageOut with FOC: +12 V forward, -6 V reverse
- Feeds fuel from intake to shooter, brake mode

### Shooter Intake
- Single TalonFX (CAN 15, Tuner X: "UpTakeChannelForShooter"), DutyCycleOut: forward constant 2.0 (saturates at 100%), reverse -0.6
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
- TalonFX (CAN 14, Tuner X: "AngleShooter"), duty cycle output with 1.0 speed multiplier
- Rotates turret to aim at targets, brake mode
- Firmware soft limits enabled: -25 to +10 rotations

### Ground Intake Pivot (raise/lower)
- **Leader**: TalonFX CAN 20 ("Right Ground Intake Raise"). The code only ever commands this one.
- **Follower**: TalonFX CAN 21 ("Left Ground Intake Raise"), set once at startup to copy the leader with `Follower(20, Opposed)` because the two are mounted mirrored
- Hold-to-move with `VoltageOut` at 0.75 V (`kMoveVolts`); release = stop. Brake mode, 20 A stator limit on both motors
- **FOC is turned off** (`withEnableFOC(false)`): these motors are not Pro-licensed, and an unlicensed Talon FX refuses FOC commands
- **No position control or soft limits yet**: nothing stops it at the end of travel except releasing the button
- Wrong direction: flip `kLeaderInverted`. Motors fighting: flip `kFollowerAlignment`. Both live in `Constants.GroundIntakePivot`
- The old ground intake **roller** code was deleted: its motor is gone and its old ID (20) is now the pivot leader

### Climb (REMOVED, Oct 2026)
- The climb motor's old ID (21) now belongs to the intake pivot follower, so the climb code was deleted (it's in git history)
- An **"Elevator"** motor (CAN 19) is on the robot but **not coded yet**

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
| `IndexSubsystem` | 1 TalonFX | DutyCycleOut | `run(speed)`, `stop()` |
| `ShooterIntakeSubsystem` | 1 TalonFX | DutyCycleOut | `run(speed)`, `stop()` |
| `GoobaSubsystem` (RETIRED, not created) | none | MotionMagicVoltage | `setPosition(rot)`, `getPosition()`, `getRotationForDistance(d)` |
| `Goober` | 1 TalonFX | DutyCycleOut | `setMotorSpeed(pct)`, `stop()` |
| `LimelightSubsystem` | Limelight camera | NetworkTables | `getTX()`, `getTY()`, `isValidTarget()`, `distanceToTarget()` |
| `MariosEar` | 2 PhotonVision + Limelight | PhotonCamera | `limelightHasTarget()`, `getLimelightTX()`, `getLeftResult()`, `getRightResult()` |
| `GroundIntakePivotSubsystem` | 2 TalonFX (20 leader, 21 follower) | VoltageOut + Follower | `raiseCommand()`, `lowerCommand()`, `setVolts(v)`, `stop()` |

## Command Architecture

Commands follow the standard WPILib command-based pattern. Key patterns:

- **`FuelHandlingCommand`**: The core shooting/intake command. Coordinates index + shooterIntake + shooter simultaneously. `isForward=true` shoots, `isForward=false` reverses/intakes (-1000 RPM). Runs while held (autos add timeouts).
- **`FeedShooterCommand`**: Runs index + shooter intake only, so the shooter can be spun up independently (operator layout).
- **`MariosEarCommand`**: Multi-phase turret aiming with priority: (1) Limelight PID tracking → (2) Left USB cam coarse search → (3) Right USB cam coarse search → (4) stop.
- **`GooberAlign`**: Limelight PID turret alignment on raw tx (camera is centered on the shooter). When no valid target is in view it sweeps the turret between its soft limits hunting for one. Never finishes on its own — bind with whileTrue or a timeout.
- **`AutoGooba`** (RETIRED with the hood): Continuous auto-aim that maps Limelight distance (m) to hood position via the interpolating shot map, with a moving-average filter on distance.
- **Instant commands**: `GoobaToggleCommand` (retired) finishes immediately after setting state.
- **While-held commands**: `ManualTurretCommand` and the intake pivot's `raiseCommand()`/`lowerCommand()` (built with `startEnd`) run while held and stop on release.

## Controller Bindings

### Driver Controller (Port 0 - Xbox)
| Input | Action |
|-------|--------|
| Left Stick | Swerve translation (X/Y) with cubic scaling |
| Right Stick X | Swerve rotation with cubic scaling |
| Start (held) | Activates button speed limiter (default 50%) |
| Right/Left Trigger | Unbound (old intake roller removed) |
| X, A | Unbound (used to toggle the removed pneumatic pistons) |
| Left Bumper | Seed field-centric heading |
| D-Pad Up (held) | Raise ground intake (0.75 V) |
| D-Pad Down (held) | Lower ground intake (0.75 V) |
| D-Pad Left | Toggle shooter idle mode |
| D-Pad Right | Toggle simulated tank drive |

### Operator Controller (Port 1 - Xbox)
| Input | Action |
|-------|--------|
| Left Trigger (held) | Spin up shooter (fast RPM) |
| Right Trigger (held) | Feed shooter (index + shooter intake) |
| B (held) | Rewind index only |
| Left Bumper (held) | Turret tracks the hub (GooberAlign) |
| Y, D-Pad Up/Down | Unbound (hood removed) |
| D-Pad Left/Right (held) | Jog turret left/right |

### HOTAS (Ports 2/3 - Logitech X56, optional)
Full mapping and setup steps in `docs/X56-HOTAS-SETUP.md`. Stick pitch/roll/twist
drive the swerve, the right throttle lever is an analog speed governor, and the
stick buttons mirror the driver Xbox layout. When the stick is unplugged all
HOTAS bindings are inert and the Xbox controller drives as normal. The main hat is
unbound now (it ran the climb, which was removed).

### Speed Limiting
- **Global speed limiter**: Shuffleboard chooser (100%/75%/50%/25%), always active
- **Button speed limiter**: Shuffleboard chooser (50%/75%/25%/10%), active only while Start is held (Xbox only)
- **HOTAS throttle**: right throttle lever scales speed 15%-100% on top of the global limiter

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
| 19 | TalonFX | Elevator | **not coded yet** |
| 20 | TalonFX | Right Ground Intake Raise | `Constants.GroundIntakePivot.kLeaderId` |
| 21 | TalonFX | Left Ground Intake Raise | `Constants.GroundIntakePivot.kFollowerId` |
| 41 | Pigeon2 | Pigeon 2 | Swerve |

All devices are on the roboRIO's built-in CAN bus (`TunerConstants.kCANBus` is `""`), no CANivore.
IDs 51 and 61 no longer exist. The retired hood code still says 51 but is never created.
**Free IDs for new devices:** 22-40, 42-50, 52+. New CTRE devices ship as ID 0, so set the ID
in Phoenix Tuner X before writing code.

## Patterns & Conventions

- **Naming**: The team uses creative nicknames for subsystem classes: "Gooba" = hood/arc mechanism, "Goober" = turret, "MariosEar" = multi-camera vision system, "Marcos" = named-command registrar. Instance variables use plain descriptive names (`limelight`, `mariosEar`, `groundIntakePivot`, ...).
- **Vision mounting**: The Limelight is mounted HORIZONTALLY and CENTERED on the shooter (2026 offseason remount) — `tx` aims the turret, `ty` feeds the distance trig. Re-measure `Constants.Limelight.kMountAngleDegrees`/`kLensHeightMeters` whenever the mount changes.
- **Hub shifts**: `HubShiftTracker.isOurHubOpen()` implements the REBUILT shift schedule (game data at teleop start + match timer). Driver rumble and the dashboards read it via `RobotContainer.isHubOpenForUs()`.
- **Constants**: All hardware IDs, PID gains, speeds, and positions live in `Constants.java` inner classes. Comments mark values as `//fixed` when confirmed on hardware.
- **Marcos.java**: Centralized PathPlanner named command registration, separate from RobotContainer. Named commands must be registered before `AutoBuilder.buildAutoChooser()`.
- **Motor control**: Phoenix 6 API throughout. TalonFX motors use `DutyCycleOut` for simple speed control, `VelocityVoltage` for RPM control (shooter), `MotionMagicVoltage` for smooth position control (retired hood), and `VoltageOut` + `Follower` for the two-motor intake pivot. A follower is set once at startup and must never be sent any other command.
- **Phoenix Pro licensing / FOC**: Phoenix 6 control requests turn FOC on by default, and FOC is a paid Pro feature. A Talon FX without a Pro license refuses FOC commands and won't move (LED alternates red/green). Check the license badge in Tuner X, and use `.withEnableFOC(false)` for unlicensed motors (the intake pivot does).
- **Simulation**: MapleSim-backed swerve physics simulation with 0.002s loop period. Simulated robot: 115 lbs, KrakenX60 motors, 1.2 tire COF.
- **Logging**: DogLog for structured logging (battery, odometry, module states). Telemetry class publishes swerve state to NetworkTables + SignalLogger. HootAutoReplay for timestamp/joystick replay.
- **Drive input**: Cubic scaling on joystick axes (`Math.pow(abs, 3) * signum`) for fine control at low speeds.
- **TunerConstants**: Auto-generated by CTRE Tuner X, contains all swerve module configurations. Do not manually edit without re-running Tuner X.

## Known TODOs / FIXMEs

- **Shooter top roller (CAN 18) is not coded**: it won't spin when shooting. Needs a direction and an RPM
- **Elevator (CAN 19) is not coded**
- **Check each motor's Pro license badge in Tuner X.** The index (13) asks for FOC explicitly, and the shooter, uptake and turret use the FOC default. Any of them that are unlicensed will refuse to move until switched to `.withEnableFOC(false)`
- **Intake pivot has no position control or soft limits yet**: record raised/lowered positions from the dashboard ("Intake Pivot Position (rot)") to add them
- Verify the turret aim sign after the Limelight remount: if the turret runs away from the target, flip the sign in `GooberAlign` (`-limelight.getTX()`)
- Re-measure `Constants.Limelight.kMountAngleDegrees` and `kLensHeightMeters` on the new camera mount — `distanceToTarget()` is wrong until then
- `Constants.Shooter.kFastTargetRPM = 7000` is clamped to `kMaxRPM = 3100` at runtime — raise `kMaxRPM` when the flywheel is trusted at higher speeds
- X56 HOTAS axis/button indices in `Constants.Hotas` need on-hardware verification (docs/X56-HOTAS-SETUP.md)
- Servo mechanism(s) replacing the pneumatics: not built yet, not coded yet (use WPILib `Servo` on a roboRIO PWM port when ready)
- Hood retired: the shot map and AutoGooba are unused until a hood comes back
