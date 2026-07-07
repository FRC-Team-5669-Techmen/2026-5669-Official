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
| Phoenix5 (legacy) | CTRE PCM for pneumatics |
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
│   ├── GoobaToggleCommand      # Deploy/stow gooba (instant)
│   ├── AutoGooba               # Auto-aim gooba using Limelight shot map
│   ├── GooberAlign             # PID turret align w/ search sweep (Limelight)
│   ├── MariosEarCommand        # Multi-phase turret aim (Limelight → USB cams)
│   ├── TogglePneumaticCommand  # Toggle a double solenoid (instant)
│   ├── ManualGoobaCommand      # D-pad jog gooba position up/down
│   ├── ManualTurretCommand     # D-pad jog turret left/right
│   ├── RunGroundIntakeCommand  # Run ground intake roller
│   └── RunClimbMotorCommand    # Run climb motor at given speed
├── subsystems/
│   ├── CommandSwerveDrivetrain # CTRE swerve + PathPlanner AutoBuilder + MapleSim
│   ├── ShooterSubsystem        # Dual TalonFX (leader/follower), velocity control
│   ├── IndexSubsystem          # Single TalonFX, duty cycle belt/roller
│   ├── ShooterIntakeSubsystem  # Single TalonFX, feeds fuel to shooter
│   ├── GoobaSubsystem          # Kraken X44, Motion Magic position (hood/arc)
│   ├── Goober                  # Turret motor, duty cycle rotation
│   ├── LimelightSubsystem      # Limelight NetworkTables (tx/ty/ta/tv/botpose)
│   ├── MariosEar               # Multi-camera vision: Limelight + 2 PhotonVision USB cams
│   ├── PneumaticSubsystem      # Reusable double solenoid (CTRE PCM)
│   ├── GroundIntakeSubsystem   # Kraken X44, ground pickup roller
│   └── ClimbSubsystem          # Kraken X60, winch motor with soft limits
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
- **Leader** TalonFX (CAN 16) + **Follower** TalonFX (CAN 17), aligned
- Velocity control (VelocityVoltage) at 5000 RPM (fast) / 2500 RPM (slow) / -1000 RPM (reverse)
- PID: kP=-0.11

### Index / Feeder
- Single TalonFX (CAN 18), duty cycle at 0.5 speed
- Feeds fuel from intake to shooter, brake mode

### Shooter Intake
- Single TalonFX (CAN 19), duty cycle at 0.6 speed
- Pulls fuel into the shooter path, brake mode

### Gooba (Hood/Arc)
- **Kraken X44** TalonFX (CAN 51)
- **Motion Magic** position control for smooth hood angle adjustment
- PID: kP=2.4, kD=0.1; cruise velocity 80 RPS, acceleration 160 RPS/s
- Positions: stowed=0.0, deployed=3.5 rotations (needs tuning)
- Has interpolating distance→rotation map (shot map) for auto-aim
- 40A supply current limit

### Goober (Turret)
- TalonFX (CAN 61), duty cycle output with 0.25 speed multiplier
- Rotates turret to aim at targets, brake mode
- Software limit switches commented out (in development)

### Ground Intake
- **Kraken X44** TalonFX (CAN 20), 75% speed, coast mode
- 40A supply current limit

### Climb
- **Kraken X60** TalonFX (CAN 21), 20% speed
- 1:100 gearbox with ratchet mechanism
- Soft limits: +/- 1000 rotations
- 60A supply current limit
- Pneumatic piston (Sol3) for climb ratchet

### Pneumatics
- CTRE PCM (CAN 25) with 3 double solenoids:
  - **Sol1** (ports 0/1): Intake piston
  - **Sol2** (ports 3/4): Intake 2 piston
  - **Sol3** (ports 2/6): Climb ratchet piston

### Vision
- **Limelight**: AprilTag detection, provides tx/ty/ta/tv/botpose via NetworkTables
  - Horizontal offset correction for off-center camera mounting (1.5" offset)
  - Distance-to-target calculation using trigonometry
- **PhotonVision USB cameras**: "Left" and "Right" cameras for wide-angle target search
- **MariosEar**: Fuses all 3 cameras into a multi-phase turret aiming pipeline

## Subsystem Responsibilities

| Subsystem | Hardware | Control Mode | Key Methods |
|-----------|----------|-------------|-------------|
| `CommandSwerveDrivetrain` | 8 TalonFX + 4 CANcoder + Pigeon2 | Field/Robot centric | `applyRequest()`, `seedFieldCentric()` |
| `ShooterSubsystem` | 2 TalonFX (leader/follower) | VelocityVoltage | `runAtRPM(rpm)`, `stop()` |
| `IndexSubsystem` | 1 TalonFX | DutyCycleOut | `run(speed)`, `stop()` |
| `ShooterIntakeSubsystem` | 1 TalonFX | DutyCycleOut | `run(speed)`, `stop()` |
| `GoobaSubsystem` | 1 TalonFX (X44) | MotionMagicVoltage | `setPosition(rot)`, `getPosition()`, `getRotationValueFromDistance(d)` |
| `Goober` | 1 TalonFX | DutyCycleOut | `setMotorSpeed(pct)`, `stop()` |
| `LimelightSubsystem` | Limelight camera | NetworkTables | `getTX()`, `getTY()`, `isTargetAvailable()`, `distanceToTarget()`, `getNewTX()` |
| `MariosEar` | 2 PhotonVision + Limelight | PhotonCamera | `limelightHasTarget()`, `getLimelightTX()`, `getLeftResult()`, `getRightResult()` |
| `PneumaticSubsystem` | DoubleSolenoid (CTRE PCM) | Solenoid toggle | `extend()`, `retract()`, `toggle()` |
| `GroundIntakeSubsystem` | 1 TalonFX (X44) | DutyCycleOut | `runIntake(speed)`, `stop()` |
| `ClimbSubsystem` | 1 TalonFX (X60) | DutyCycleOut | `runMotor(speed)`, `stop()` |

## Command Architecture

Commands follow the standard WPILib command-based pattern. Key patterns:

- **`FuelHandlingCommand`**: The core shooting/intake command. Coordinates index + shooterIntake + shooter simultaneously. `isForward=true` shoots, `isForward=false` reverses/intakes (-1000 RPM). Runs while held (autos add timeouts).
- **`FeedShooterCommand`**: Runs index + shooter intake only, so the shooter can be spun up independently (operator layout).
- **`MariosEarCommand`**: Multi-phase turret aiming with priority: (1) Limelight PID tracking → (2) Left USB cam coarse search → (3) Right USB cam coarse search → (4) stop.
- **`GooberAlign`**: Limelight PID turret alignment using corrected tx with horizontal offset compensation. When no valid target is in view it sweeps the turret between its soft limits hunting for one. Never finishes on its own — bind with whileTrue or a timeout.
- **`AutoGooba`**: Continuous auto-aim that maps Limelight tx to hood position via interpolating tree map.
- **Instant commands**: `GoobaToggleCommand`, `TogglePneumaticCommand` finish immediately after setting state.
- **While-held commands**: `ManualGoobaCommand`, `ManualTurretCommand`, `RunClimbMotorCommand`, `RunGroundIntakeCommand` run continuously and stop on release.

## Controller Bindings

### Driver Controller (Port 0 - Xbox)
| Input | Action |
|-------|--------|
| Left Stick | Swerve translation (X/Y) with cubic scaling |
| Right Stick X | Swerve rotation with cubic scaling |
| Start (held) | Activates button speed limiter (default 50%) |
| Right Trigger (held) | Ground intake in |
| Left Trigger (held) | Ground intake reverse (spit out) |
| X | Toggle intake pistons |
| A | Toggle climb piston |
| Left Bumper | Seed field-centric heading |
| D-Pad Up/Down (held) | Climb motor up/down |
| D-Pad Left | Toggle shooter idle mode |
| D-Pad Right | Toggle simulated tank drive |

### Operator Controller (Port 1 - Xbox)
| Input | Action |
|-------|--------|
| Left Trigger (held) | Spin up shooter (fast RPM) |
| Right Trigger (held) | Feed shooter (index + shooter intake) |
| B (held) | Rewind index only |
| Y | Toggle gooba stowed/deployed |
| Left Bumper (held) | Track hub: GooberAlign + AutoGooba |
| D-Pad Up (held) | Jog gooba toward stow |
| D-Pad Down (held) | Jog gooba toward deploy |
| D-Pad Left/Right (held) | Jog turret left/right |

### HOTAS (Ports 2/3 - Logitech X56, optional)
Full mapping and setup steps in `docs/X56-HOTAS-SETUP.md`. Stick pitch/roll/twist
drive the swerve, the right throttle lever is an analog speed governor, and the
stick buttons mirror the driver Xbox layout. When the stick is unplugged all
HOTAS bindings are inert and the Xbox controller drives as normal.

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
| `visionScore` | (GooberAlign ∥ AutoGooba) → spin up → shoot → stop | staged |
| `shoot` | FuelHandlingCommand (forward) | 2.0s |
| `stopShooter` | InstantCommand → shooter.stop() | - |
| `intake` | FuelHandlingCommand (reverse) | 2.0s |
| `lowerIntake` | InstantCommand → intake pistons extend | - |
| `runGroundIntake` | RunGroundIntakeCommand (also bound as EventTrigger) | none |
| `deployGooba` | GoobaToggleCommand (deploy) | - |
| `stowGooba` | GoobaToggleCommand (stow) | - |
| `autoAimGooba` | AutoGooba (vision-based) | 2.0s |
| `aimTurret` | MariosEarCommand (multi-cam) | 2.0s |
| `alignTurret` | GooberAlign (Limelight-only) | 2.0s |
| `togglePiston1` | TogglePneumaticCommand (intake pistons) | - |
| `togglePiston2` | TogglePneumaticCommand (climb piston) | - |

### Paths & Autos
- **Autos**: `Center Preload - Dumb`, `Center Preload - Vision`, `Left 3-Piece Auto`, `Right 3-Piece Auto`
- **Paths**: see `src/main/deploy/pathplanner/paths/`

## CAN ID Map

| ID | Device | Subsystem |
|----|--------|-----------|
| 1 | TalonFX (Back Right Steer) | Swerve |
| 2 | TalonFX (Front Right Steer) | Swerve |
| 3 | TalonFX (Back Left Drive) | Swerve |
| 4 | TalonFX (Back Right Drive) | Swerve |
| 5 | TalonFX (Front Left Steer) | Swerve |
| 6 | TalonFX (Front Left Drive) | Swerve |
| 7 | TalonFX (Back Left Steer) | Swerve |
| 8 | TalonFX (Front Right Drive) | Swerve |
| 9 | CANcoder (Front Right) | Swerve |
| 10 | CANcoder (Front Left) | Swerve |
| 11 | CANcoder (Back Left) | Swerve |
| 12 | CANcoder (Back Right) | Swerve |
| 16 | TalonFX (Shooter Leader) | Shooter |
| 17 | TalonFX (Shooter Follower) | Shooter |
| 18 | TalonFX (Index) | Index |
| 19 | TalonFX (Shooter Intake) | Shooter Intake |
| 20 | TalonFX (Ground Intake) | Ground Intake |
| 21 | TalonFX (Climb) | Climb |
| 25 | CTRE PCM | Pneumatics |
| 41 | Pigeon2 IMU | Swerve |
| 51 | TalonFX (Gooba/Hood) | Gooba |
| 61 | TalonFX (Goober/Turret) | Turret |

## Patterns & Conventions

- **Naming**: The team uses creative nicknames for subsystem classes: "Gooba" = hood/arc mechanism, "Goober" = turret, "MariosEar" = multi-camera vision system, "Marcos" = named-command registrar. Instance variables use plain descriptive names (`limelight`, `mariosEar`, `climbPiston`, ...).
- **Vision mounting**: The Limelight is mounted SIDEWAYS — raw `tx` is the vertical angle (used for distance), raw `ty` is the horizontal angle (used for turret aim). See `LimelightSubsystem` javadoc before touching vision math.
- **Hub shifts**: `HubShiftTracker.isOurHubOpen()` implements the REBUILT shift schedule (game data at teleop start + match timer). Driver rumble and the dashboards read it via `RobotContainer.isHubOpenForUs()`.
- **Constants**: All hardware IDs, PID gains, speeds, and positions live in `Constants.java` inner classes. Comments mark values as `//fixed` when confirmed on hardware.
- **Marcos.java**: Centralized PathPlanner named command registration, separate from RobotContainer. Named commands must be registered before `AutoBuilder.buildAutoChooser()`.
- **Motor control**: Phoenix 6 API throughout. TalonFX motors use `DutyCycleOut` for simple speed control, `VelocityVoltage` for RPM control (shooter), and `MotionMagicVoltage` for smooth position control (gooba).
- **Pneumatics**: `PneumaticSubsystem` is a reusable class instantiated multiple times with different solenoid channel pairs.
- **Simulation**: MapleSim-backed swerve physics simulation with 0.002s loop period. Simulated robot: 115 lbs, KrakenX60 motors, 1.2 tire COF.
- **Logging**: DogLog for structured logging (battery, odometry, module states). Telemetry class publishes swerve state to NetworkTables + SignalLogger. HootAutoReplay for timestamp/joystick replay.
- **Drive input**: Cubic scaling on joystick axes (`Math.pow(abs, 3) * signum`) for fine control at low speeds.
- **TunerConstants**: Auto-generated by CTRE Tuner X, contains all swerve module configurations. Do not manually edit without re-running Tuner X.

## Known TODOs / FIXMEs

- Gooba shot map has 4 calibration points — re-record whenever the hood or camera mount changes
- `Constants.Shooter.kFastTargetRPM = 7000` is clamped to `kMaxRPM = 3100` at runtime — raise `kMaxRPM` when the flywheel is trusted at higher speeds
- `Constants.GroundIntake.kSupplyCurrentLimit = 40` is NOT what the subsystem applies (hardcoded 60A supply / 80A stator after breaker-trip testing — see git history before changing)
- X56 HOTAS axis/button indices in `Constants.Hotas` need on-hardware verification (docs/X56-HOTAS-SETUP.md)
- Climb soft limits (1 to 150 rotations) mean the climb cannot run down until it has first risen above 1 rotation after boot
