// Copyright (c) FIRST and other WPILib contributors.
// SigmaAura-est was here...

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.controls.HotasX56;
import frc.robot.generated.TunerConstants;

// Subsystems
import frc.robot.characterization.DriveTests;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.AimSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DefenseMode;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Goober;
import frc.robot.subsystems.GoobaSubsystem;
import frc.robot.subsystems.GroundIntakePivotSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LocalizationCheck;
import frc.robot.subsystems.PitCheck;
import frc.robot.subsystems.ShotMapGuide;
import frc.robot.subsystems.BackupShots;
import frc.robot.subsystems.ShooterIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionLocalizer;

// Commands
import frc.robot.commands.AimCommand;
import frc.robot.commands.VacuumCommand;
import frc.robot.commands.FeedShooterCommand;
import frc.robot.commands.ManualGoobaCommand;
import frc.robot.commands.ManualTurretCommand;
import frc.robot.commands.RunShooterCommand;
import frc.robot.commands.ShootCommand;

public class RobotContainer {

    private final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double maxAngularRate = Constants.Operator.kMaxAngularRate.in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * Constants.Operator.kDeadband)
            .withRotationalDeadband(maxAngularRate * Constants.Operator.kRotationalDeadband)
            .withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDeadband(maxSpeed * Constants.Operator.kDeadband)
            .withRotationalDeadband(maxAngularRate * Constants.Operator.kRotationalDeadband)
            .withDriveRequestType(DriveRequestType.Velocity);

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3.0);
    // Defense mode: quicker response (both sets run every loop so switching never jumps).
    private final SlewRateLimiter xFast = new SlewRateLimiter(Constants.Defense.kSlewRate);
    private final SlewRateLimiter yFast = new SlewRateLimiter(Constants.Defense.kSlewRate);
    private final SlewRateLimiter rotFast = new SlewRateLimiter(Constants.Defense.kSlewRate);
    // Defense mode, sticks centered: wheels in an X so the robot is hard to push.
    private final SwerveRequest.SwerveDriveBrake xLock = new SwerveRequest.SwerveDriveBrake();

    private final SendableChooser<String> driveModeChooser = new SendableChooser<>();
    private final SendableChooser<Double> globalSpeedLimiter = new SendableChooser<>();
    private final SendableChooser<Double> buttonSpeedLimiter = new SendableChooser<>();
    private final Telemetry logger = new Telemetry(maxSpeed);

    public final CommandXboxController driverController =
        new CommandXboxController(Constants.Operator.kDriverControllerPort);
    public final CommandXboxController operatorController =
        new CommandXboxController(Constants.Operator.kOperatorControllerPort);

    // Logitech X56 flight stick + throttle. Optional: when unplugged, every HOTAS
    // binding is inert and the Xbox driver controller behaves exactly as before.
    public final HotasX56 hotas = new HotasX56(Constants.Hotas.kStickPort, Constants.Hotas.kThrottlePort);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final ShooterSubsystem shooter = new ShooterSubsystem();
    public final IndexSubsystem index = new IndexSubsystem();
    public final ShooterIntakeSubsystem shooterIntake = new ShooterIntakeSubsystem();
    public final Goober goober = new Goober(); // turret, CAN 14 ("AngleShooter")
    public final LimelightSubsystem limelight = new LimelightSubsystem();
    public final GroundIntakePivotSubsystem groundIntakePivot = new GroundIntakePivotSubsystem();
    public final ElevatorSubsystem elevator = new ElevatorSubsystem();  // climber, CAN 19
    public final IntakeRollerSubsystem intakeRoller = new IntakeRollerSubsystem(); // ground intake roller, CAN 22
    public final GoobaSubsystem gooba = new GoobaSubsystem(); // hood, CAN 18 ("Shooter Top")
    // Automatic agitation while shooting (pump the intake / shake + unjam the spindexer). No buttons.
    public final Agitator agitator = new Agitator(shooter, index, groundIntakePivot, intakeRoller);

    // Shoot from anywhere: field position from the Limelight (camera on the turret), and the
    // shot brain that turns position into turret angle, hood and shooter speed.
    public final VisionLocalizer localizer = new VisionLocalizer(drivetrain, goober, gooba);
    public final AimSubsystem aim = new AimSubsystem(drivetrain, localizer);
    // Beginner test helpers (docs/LOCALIZATION-TESTING.md): "Localization Step", "Aim Check", "Test: Turret Sweep".
    public final LocalizationCheck localizationCheck = new LocalizationCheck(drivetrain, goober, limelight, localizer);
    // Self-check once a second: dashboard "Pit Check" = ALL GO or what is wrong. Read it before every match.
    public final PitCheck pitCheck = new PitCheck(goober, localizer);
    // Shot Map Day guide: "Shot Map Step" says what to do next; every shot goes to shot-log.csv.
    public final ShotMapGuide shotMapGuide = new ShotMapGuide(aim, shooter, gooba);
    // One shared auto-aim command: Operator RB toggles it, the backup-shot buttons turn it on.
    private final AimCommand aimCommand = new AimCommand(aim, goober, gooba, limelight, drivetrain);
    private final VacuumCommand vacuumCommand = new VacuumCommand(aim, shooter, index, shooterIntake,
        goober, gooba, groundIntakePivot, intakeRoller, aimCommand);
    public final BackupShots backupShots = new BackupShots(drivetrain, localizer);
    // Defense mode (Driver Y): intake up, hood down, quick drive, X-lock, pin + tower warnings.
    public final DefenseMode defense = new DefenseMode(drivetrain, aim);

    // Not created on purpose: MariosEar (the 2 PhotonVision USB cameras), since the
    // Limelight is the only camera.

    public final DriverDashboard driverDashboard = new DriverDashboard();

    private final SendableChooser<Command> autoChooser;

    // State Toggles
    private boolean m_isShooterIdle = false;
    private static final String kAutoLookKey = "Auto: Turret Watches Hub";
    private final SendableChooser<HubShiftTracker.AutoWinner> autoWinnerChooser = new SendableChooser<>();

    public RobotContainer() {
        // We intentionally configure more HIDs (Xbox x2 + HOTAS x2) than are usually
        // plugged in, so don't spam the DS console about the empty ports.
        DriverStation.silenceJoystickConnectionWarning(true);

        Marcos.registerNamedCommands(
            shooter, index, shooterIntake, goober, limelight, gooba, aim, drivetrain,
            groundIntakePivot, intakeRoller
        );

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        driveModeChooser.setDefaultOption("Field Centric", "field");
        driveModeChooser.addOption("Robot Centric", "robot");
        SmartDashboard.putData("Drive Mode", driveModeChooser);

        globalSpeedLimiter.setDefaultOption("100%", 1.0);
        globalSpeedLimiter.addOption("75%", 0.75);
        globalSpeedLimiter.addOption("50%", 0.5);
        globalSpeedLimiter.addOption("25%", 0.25);
        SmartDashboard.putData("Global Speed Limit", globalSpeedLimiter);

        buttonSpeedLimiter.setDefaultOption("50%", 0.5);
        buttonSpeedLimiter.addOption("75%", 0.75);
        buttonSpeedLimiter.addOption("25%", 0.25);
        buttonSpeedLimiter.addOption("10%", 0.1);
        SmartDashboard.putData("Button Speed Limit", buttonSpeedLimiter);

        configureDriveDefaultCommand();
        configureDriverBindings();
        configureOperatorBindings();
        configureHotasBindings();
        configureSystemDefaults();

        FollowPathCommand.warmupCommand().schedule();
    }

    // ==========================================
    // --- DRIVE INPUT (Xbox or HOTAS) ---
    // ==========================================
    private void configureDriveDefaultCommand() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double xInput;
                double yInput;
                double rInput;
                double speedMultiplier = globalSpeedLimiter.getSelected();

                if (hotas.isConnected()) {
                    // Flight stick: pitch = forward/back, roll = strafe, twist = rotation.
                    // The physical throttle lever scales speed on top of the global limit.
                    xInput = hotas.getForward();
                    yInput = hotas.getStrafe();
                    rInput = hotas.getTwist();
                    speedMultiplier *= hotas.getThrottleScale();
                } else {
                    xInput = -driverController.getLeftY();  // Forward/Backward
                    yInput = -driverController.getLeftX();  // Strafe Left/Right
                    rInput = -driverController.getRightX(); // Rotation

                    // Start button swaps in the alternate (usually slower) speed limit
                    if (driverController.getHID().getStartButton()) {
                        speedMultiplier = buttonSpeedLimiter.getSelected();
                    }
                }

                // Cubic curve for fine control at low speed, then slew-rate limiting
                double cx = Math.signum(xInput) * Math.pow(Math.abs(xInput), 3);
                double cy = Math.signum(yInput) * Math.pow(Math.abs(yInput), 3);
                double cr = Math.signum(rInput) * Math.pow(Math.abs(rInput), 3);
                double scaledX = xLimiter.calculate(cx);
                double scaledY = yLimiter.calculate(cy);
                double scaledRot = rotLimiter.calculate(cr);
                double fastX = xFast.calculate(cx), fastY = yFast.calculate(cy), fastRot = rotFast.calculate(cr);
                boolean defending = DefenseMode.active();
                DefenseMode.reportDriverStick(Math.hypot(xInput, yInput));
                if (defending) {
                    scaledX = fastX;
                    scaledY = fastY;
                    scaledRot = fastRot;
                    // Sticks centered and the robot nearly stopped: lock the wheels in an X (only
                    // once slow, so a full-speed stick release doesn't snap the modules sideways).
                    var st = drivetrain.getState().Speeds;
                    boolean centered = Math.abs(xInput) < Constants.Operator.kDeadband
                        && Math.abs(yInput) < Constants.Operator.kDeadband
                        && Math.abs(rInput) < Constants.Operator.kRotationalDeadband;
                    if (centered && Math.hypot(st.vxMetersPerSecond, st.vyMetersPerSecond) < 0.3) {
                        Agitator.reportDriverSticks(xInput, yInput, rInput);
                        return xLock;
                    }
                }

                double currentMaxSpeed = maxSpeed * speedMultiplier;
                double currentMaxAngularRate = maxAngularRate * speedMultiplier;
                double rotationRate = scaledRot * currentMaxAngularRate;

                // SHOOT FROM ANYWHERE heading assist: with auto-aim on and the hub behind the
                // turret (its ~90 degree dead zone), turn the robot just enough to bring the hub in
                // reach. The driver's rotate stick always wins.
                if (!defending && aim.isActive() && aim.poseUsable() && !aim.solution().reachable()
                        && !ElevatorSubsystem.isRaised()   // climbing: never turn the robot by itself
                        && Math.abs(rInput) < Constants.Aim.kDriverRotateOverride) {
                    rotationRate = MathUtil.clamp(
                        Constants.Aim.kHeadingAssistP * aim.solution().headingErrorRad(),
                        -Constants.Aim.kHeadingAssistMax, Constants.Aim.kHeadingAssistMax);
                }

                // AUTOMATIC AGITATION shimmy (dashboard "Agitate Drive Shimmy", off by default):
                // a short robot-relative forward/back shuffle to break a stuck pile. Only while the
                // driver's sticks are centered; touching a stick cancels it at once.
                Agitator.reportDriverSticks(xInput, yInput, rInput);
                if (Agitator.shimmying()) {
                    return robotCentricDrive
                        .withVelocityX(Agitator.shimmySpeed())
                        .withVelocityY(0)
                        .withRotationalRate(rotationRate);
                }

                // --- SWERVE DRIVE LOGIC ---
                if ("robot".equals(driveModeChooser.getSelected())) {
                    return robotCentricDrive
                        .withVelocityX(scaledX * currentMaxSpeed)
                        .withVelocityY(scaledY * currentMaxSpeed)
                        .withRotationalRate(rotationRate);
                }

                return fieldCentricDrive
                    .withVelocityX(scaledX * currentMaxSpeed)
                    .withVelocityY(scaledY * currentMaxSpeed)
                    .withRotationalRate(rotationRate);
            })
        );
    }

    // ==========================================
    // --- DRIVER CONTROLLER (PORT 0 - XBOX) ---
    // ==========================================
    // Matches the drive team's official map. LOCKED = practiced, don't change.
    // (Left Stick = translate, Right Stick = rotate, Start = slow mode are in the
    // drive command above.)
    private void configureDriverBindings() {
        // LOCKED Left Bumper: reset gyro / seed field centric
        // Shoot from anywhere: this also tells the robot its heading matches the field, so press it
        // with the robot pointing straight AWAY from your driver station.
        driverController.leftBumper().onTrue(drivetrain.runOnce(() -> {
            drivetrain.seedFieldCentric();
            localizer.markHeadingKnown();
        }));

        // LOCKED Right Trigger (hold): ground intake roller. Does nothing until the
        // roller is on CAN and kIntakeRollerEnabled = true.
        driverController.rightTrigger().whileTrue(intakeRoller.runCommand());

        // Left Trigger (hold): ground intake roller BACKWARDS, to spit balls out.
        // Added Oct 25 by the programming lead (was unused).
        driverController.leftTrigger().whileTrue(intakeRoller.reverseCommand());

        // LOCKED A (press): toggle ground intake DOWN / UP. Does nothing until
        // kDownPosition is set.
        driverController.a().onTrue(groundIntakePivot.toggleCommand());

        // LOCKED X: reserved (was the climb piston). Bind nothing.

        // Y (press): DEFENSE MODE toggle (added Oct 27). On: auto-aim and vacuum off, intake UP,
        // hood DOWN, quicker drive response, wheels lock in an X when the sticks are centered,
        // pin (G418) and tower (G420) warnings. See subsystems/DefenseMode.
        driverController.y().onTrue(defense.toggleCommand(() -> {
            vacuumCommand.cancel();
            aimCommand.cancel();
            groundIntakePivot.stowIfDown();
            CommandScheduler.getInstance().schedule(gooba.stowCommand());
        }));

        // LOCKED D-pad Up/Down (hold): climb up / down
        driverController.povUp().whileTrue(elevator.upCommand());
        driverController.povDown().whileTrue(elevator.downCommand());

        // LOCKED D-pad Left (press): shooter idle at 2000 RPM. Press again to turn it off.
        driverController.povLeft().onTrue(new InstantCommand(this::toggleShooterIdleMode));
    }

    // ==========================================
    // --- OPERATOR CONTROLLER (PORT 1 - XBOX) ---
    // ==========================================
    private void configureOperatorBindings() {
        // LOCKED Right Trigger (hold): shoot.
        //   Auto-aim ON (RB): feeds ONLY when ready (turret on the hub, shooter at the table's speed,
        //     hood in place), and pauses by itself when not. Hold it and keep driving.
        //   Auto-aim OFF: feeds right away, as before (spin up with LT first).
        Trigger aimOn = new Trigger(aim::isActive);
        operatorController.rightTrigger().and(aimOn).whileTrue(
            new ShootCommand(aim, shooter, index, shooterIntake, goober, gooba, false));
        operatorController.rightTrigger().and(aimOn.negate()).whileTrue(new FeedShooterCommand(index, shooterIntake));

        // VACUUM MODE (Back, press to toggle): intake + shoot hands-free. Intake down, roller in,
        // auto-aim on, fires by itself whenever ready. Scores from inside our alliance zone,
        // PASSES home from the neutral zone (follows the Start target mode). See VacuumCommand.
        // Operator RT/B and the jogs take over only while held; vacuum resumes after.
        operatorController.back().toggleOnTrue(vacuumCommand);
        // RB (aim off) and Y (manual hood) mean "I'm taking over": vacuum mode turns off.
        operatorController.rightBumper().or(operatorController.y())
            .onTrue(Commands.runOnce(vacuumCommand::cancel));

        // SHOT TARGET MODE (Start, press): AUTO -> SCORE only -> PASS only -> AUTO. Dashboard
        // "Shot Target" shows it. AUTO = score inside our alliance zone, pass from past it.
        operatorController.start().onTrue(aim.cycleModeCommand());

        // LOCKED Left Trigger (hold): shooter windup. Auto-aim ON: to the shot table's speed for
        // the distance (the shooter's default command does it, see configureSystemDefaults).
        // Auto-aim OFF: the fast preset, as before.
        operatorController.leftTrigger().and(aimOn.negate())
            .whileTrue(new RunShooterCommand(shooter, Constants.Shooter.kFastTargetRPM));

        // LOCKED Right Bumper (press): toggle Limelight auto-aim. First press starts
        // turret tracking, next press stops it. With no tag in view, the turret sweeps
        // to look for one. Using the turret jog also stops the auto-aim. Does nothing
        // until the turret's far end is set ("Set Turret Far End" dashboard button).
        // The hood (AutoGooba) follows the tag too. Using the turret jog, the hood jog or Y
        // stops the whole auto-aim (turret and hood).
        // SHOOT FROM ANYWHERE (Oct 26): aims from the robot's field position (falls back to the
        // hub tag when the position isn't known). See AimCommand.
        operatorController.rightBumper().toggleOnTrue(aimCommand);

        // BACKUP SHOTS (camera-proof): park on the spot, press its button, then hold RT.
        // A = Midshot (between hub and climb), X = Left trench, LB = Right trench (left/right as
        // seen from our driver station). See subsystems/BackupShots.
        operatorController.a().onTrue(backupShots.useCommand(BackupShots.Spot.MIDSHOT, aimCommand));
        // The trench spots sit right next to a trench: these also turn on the trench-shot override
        // (hood may go up while the robot stays put).
        operatorController.x().onTrue(backupShots.useCommand(BackupShots.Spot.LEFT_TRENCH, aimCommand)
            .andThen(aim.trenchOverrideCommand()));
        operatorController.leftBumper().onTrue(backupShots.useCommand(BackupShots.Spot.RIGHT_TRENCH, aimCommand)
            .andThen(aim.trenchOverrideCommand()));

        // TRENCH GUARD (automatic): near a trench the hood goes all the way DOWN (the Limelight on
        // the hood is taller than the trench bar with the hood up). Right stick CLICK (R3) = trench
        // shot: hood may go up while the robot stays put next to the trench; moving ends it.
        operatorController.rightStick().onTrue(aim.trenchOverrideCommand());

        // LOCKED Y (press): toggle the hood between stowed and deployed (the drive team's
        // map: "Toggle Gooba Hood"). Back now that the hood motor (18) is.
        operatorController.y().onTrue(gooba.toggleCommand());

        // D-pad Up/Down (hold): jog the hood slowly (programming lead, Oct 25; same as last
        // season). Write down "Hood Position (rot)" for good shots.
        operatorController.povUp().whileTrue(new ManualGoobaCommand(gooba, true));
        operatorController.povDown().whileTrue(new ManualGoobaCommand(gooba, false));

        // Left Stick up/down (hold): jog the ground intake slowly. Manual backup for when the
        // Driver A toggle can't do it (ignores the intake's soft limits while held). Was the D-pad.
        new Trigger(() -> operatorController.getLeftY() < -0.5).whileTrue(groundIntakePivot.jogUpCommand());
        new Trigger(() -> operatorController.getLeftY() > 0.5).whileTrue(groundIntakePivot.jogDownCommand());

        // OPEN D-pad Left/Right (hold): manual turret jog. Right = turn right (away from HOME).
        // The diagonals count too: a D-pad often reads Right as Up-Right or Down-Right for a moment,
        // and with plain Left/Right that stopped and restarted the turret (it stuttered).
        Trigger turretJogLeft = operatorController.povLeft()
            .or(operatorController.povUpLeft()).or(operatorController.povDownLeft());
        Trigger turretJogRight = operatorController.povRight()
            .or(operatorController.povUpRight()).or(operatorController.povDownRight());
        turretJogLeft.whileTrue(new ManualTurretCommand(goober, -Constants.Turret.kManualJogSpeed));
        turretJogRight.whileTrue(new ManualTurretCommand(goober, Constants.Turret.kManualJogSpeed));

        // Not on the map, kept on purpose: B (hold) runs the Spindex backwards to clear a jam
        operatorController.b().whileTrue(new StartEndCommand(
            () -> index.run(Constants.Index.kReverseSpeed),
            index::stop,
            index
        ));
    }

    // ==========================================
    // --- HOTAS (PORTS 2 & 3 - X56 STICK + THROTTLE) ---
    // ==========================================
    // SWITCHED OFF (Constants.Hotas.kEnabled = false): only the two Xbox controllers
    // drive. The HOTAS can never use ports 0 or 1. See docs/X56-HOTAS-SETUP.md.
    private void configureHotasBindings() {
        if (!Constants.Hotas.kEnabled) {
            return;
        }
        Trigger hotasConnected = new Trigger(hotas::isConnected);

        // Pinkie lever: re-zero field-centric heading. All other buttons are unbound.
        hotas.pinkie().and(hotasConnected).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    }

    // ==========================================
    // --- SYSTEM DEFAULTS ---
    // ==========================================
    private void configureSystemDefaults() {
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // When the robot disables, drop the intake's Motion Magic target. Otherwise the
        // motor would remember it and could move on its own when re-enabled.
        RobotModeTriggers.disabled().onTrue(groundIntakePivot.stopCommand());
        // Same for the hood: when re-enabled it stays put until a button moves it.
        RobotModeTriggers.disabled().onTrue(gooba.stopCommand());

        // Dashboard button for when the code restarts with the intake somewhere other than UP.
        SmartDashboard.putData("Zero Intake (intake is UP)", groundIntakePivot.zeroCommand());

        // Elevator: zero button (elevator all the way down), and a recovery switch that turns
        // the soft limits off in case it was powered on with the elevator up.
        SmartDashboard.putData("Zero Elevator (elevator is DOWN)", elevator.zeroCommand());
        SmartDashboard.putData("Elevator Limits OFF (recovery)", elevator.limitsOffCommand());

        // Turret: zero button, only for when the turret is sitting exactly on its HOME mark.
        SmartDashboard.putData("Zero Turret (turret at HOME)", goober.zeroCommand());
        // Turret 270° limit: jog to the far end and press Set. Clear forgets it (to re-measure).
        SmartDashboard.putData("Set Turret Far End (turret at far end)", goober.setEndCommand());
        SmartDashboard.putData("Clear Turret Far End", goober.clearEndCommand());

        // Hood: zero button, for when the hood is sitting all the way DOWN.
        SmartDashboard.putData("Zero Hood (hood is DOWN)", gooba.zeroCommand());
        // Hood measuring switch: click = limits off, click again = back on.
        SmartDashboard.putData("Hood Limits OFF (measure)", gooba.limitsOffCommand());

        // AUTO LOCALIZATION: during autos the turret (which carries the Limelight) keeps pointing at
        // our hub while the robot drives, so the camera sees hub tags the whole time and keeps
        // correcting the position (wheel slip and bumps make odometry drift). Only the turret moves,
        // not the hood (vision is skipped while the hood moves). "score" takes the turret over as
        // usual; this resumes after. Dashboard switch "Auto: Turret Watches Hub" (saved).
        Preferences.initBoolean(kAutoLookKey, true);
        SmartDashboard.putBoolean(kAutoLookKey, Preferences.getBoolean(kAutoLookKey, true));
        goober.setDefaultCommand(Commands.run(() -> {
            boolean on = SmartDashboard.getBoolean(kAutoLookKey, true);
            if (on != Preferences.getBoolean(kAutoLookKey, true)) Preferences.setBoolean(kAutoLookKey, on);
            if (on && DriverStation.isAutonomousEnabled() && aim.poseUsable() && goober.endMeasured()) {
                var s = aim.hubSolution(); // our hub, no shoot-on-the-move lead (it's for the camera)
                goober.aimAtAngleWithRate(s.turretDegrees(), s.reachable() ? s.turretRateDegPerSec()
                    : Math.toDegrees(drivetrain.getState().Speeds.omegaRadiansPerSecond));
            } else {
                goober.stop();
                goober.clearAngleTarget();
            }
        }, goober).withName("Turret: watch hub in auto"));

        shooter.setDefaultCommand(new RunCommand(() -> {
            boolean wantsShot = operatorController.getHID().getRightTriggerAxis() > 0.5
                || operatorController.getHID().getLeftTriggerAxis() > 0.5
                || aim.vacuum();   // vacuum mode: keep the wheel up even while an unjam has the feeders
            if (aim.isActive() && wantsShot) {
                shooter.runAtRPM(aim.targetRpm()); // auto-aim: table speed while LT or RT is held
            } else if (m_isShooterIdle) {
                shooter.runAtRPM(Constants.Shooter.kIdleRPM);
            } else {
                shooter.stop();
            }
        }, shooter));

        // Shot table tuning (see AimSubsystem): record the "Tune" values at the current distance.
        SmartDashboard.putData("Record Shot Point", aim.recordPointCommand());
        SmartDashboard.putData("Test: Turret Sweep", localizationCheck.turretSweepTest());
        SmartDashboard.putData("Re-find Position (camera)", localizer.refindCommand());
        SmartDashboard.putData("Reset Shot Stats", shooter.resetShotStatsCommand());
        SmartDashboard.putData("Set Pose: Plot Spot", backupShots.plotSpotCommand());
        // AUTO WINNER override: only if the FMS game data never shows up ("Our Hub" stays unknown
        // in a real match, both controllers buzz at the start of teleop). Leave on FMS otherwise.
        autoWinnerChooser.setDefaultOption("FMS (automatic)", HubShiftTracker.AutoWinner.FMS);
        autoWinnerChooser.addOption("WE won auto", HubShiftTracker.AutoWinner.WE_WON);
        autoWinnerChooser.addOption("THEY won auto", HubShiftTracker.AutoWinner.THEY_WON);
        autoWinnerChooser.onChange(HubShiftTracker::setAutoWinnerOverride);
        SmartDashboard.putData("Auto Winner", autoWinnerChooser);
        SmartDashboard.putData("Save Spot: Midshot", backupShots.saveCommand(BackupShots.Spot.MIDSHOT));
        SmartDashboard.putData("Save Spot: Left Trench", backupShots.saveCommand(BackupShots.Spot.LEFT_TRENCH));
        SmartDashboard.putData("Save Spot: Right Trench", backupShots.saveCommand(BackupShots.Spot.RIGHT_TRENCH));
        SmartDashboard.putData("Clear Recorded Shot Points", aim.clearRecordedCommand());
        SmartDashboard.putData("Clear Recorded Pass Points", aim.clearRecordedPassCommand());
        SmartDashboard.putData("Clear Recorded Climb Points", aim.clearRecordedClimbCommand());
        // Drive tests (enabled, teleop, clear space): results on "Drive Test" + console. See DriveTests.
        // Read the sticks directly: a test takes over the drivetrain, so the drive command's report freezes.
        DriveTests driveTests = new DriveTests(drivetrain, () -> Math.max(
            Math.hypot(driverController.getLeftX(), driverController.getLeftY()),
            Math.abs(driverController.getRightX())));
        SmartDashboard.putData("Test: Wheel Radius", driveTests.wheelRadiusCommand());
        SmartDashboard.putData("Test: Drive Feedforward", driveTests.feedforwardCommand());
        SmartDashboard.putData("Test: Slip Current", driveTests.slipCurrentCommand());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void toggleShooterIdleMode() {
        m_isShooterIdle = !m_isShooterIdle;
        SmartDashboard.putBoolean("Shooter Idle Active", m_isShooterIdle);
        System.out.println("Shooter Idle State Toggled: " + m_isShooterIdle);
    }

    /** An auto is starting: its path sets the field position, so the heading is known. */
    public void onAutoStart() {
        localizer.markPoseSetByAuto();
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * True when scoring into our HUB earns points right now.
     * Full REBUILT shift logic lives in {@link HubShiftTracker}.
     */
    public boolean isHubOpenForUs() {
        return HubShiftTracker.isOurHubOpen();
    }

    public void updateDashboard() {
        boolean isHubOpen = isHubOpenForUs();

        driverDashboard.updateLiveStats(
            limelight.getTX(),
            limelight.getTY(),
            limelight.isTargetAvailable(),
            isHubOpen,
            shooter.getCurrentRpm());

        SmartDashboard.putBoolean("HUB OPEN", isHubOpen);

        // Operator feel: a firm buzz while auto-aim is on and everything is ready to fire.
        boolean ready = DriverStation.isTeleopEnabled() && aim.readyToFire(goober, shooter, gooba);
        operatorController.getHID().setRumble(RumbleType.kBothRumble, ready ? 0.4 : 0.0);
        SmartDashboard.putBoolean("Ready To Fire", ready);
        // Which Xbox controller is which: slot 0 = DRIVER, slot 1 = OPERATOR in the
        // Driver Station USB tab. Driver A / Right Trigger / D-pad only work from slot 0.
        SmartDashboard.putBoolean("Driver Controller (slot 0)", DriverStation.isJoystickConnected(0));
        SmartDashboard.putBoolean("Operator Controller (slot 1)", DriverStation.isJoystickConnected(1));
        SmartDashboard.putBoolean("HOTAS Connected", hotas.isConnected());
        SmartDashboard.putBoolean("HOTAS Throttle Connected", hotas.isThrottleConnected());
    }
}
