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
import frc.robot.controls.Haptics;
import frc.robot.generated.TunerConstants;

// Subsystems
import frc.robot.characterization.DriveTests;
import frc.robot.subsystems.PowerBudget;
import frc.robot.subsystems.PracticeMatch;
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
            .withDeadband(Constants.Operator.kRequestDeadbandMps)
            .withRotationalDeadband(Constants.Operator.kRequestDeadbandRadPs)
            .withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDeadband(Constants.Operator.kRequestDeadbandMps)
            .withRotationalDeadband(Constants.Operator.kRequestDeadbandRadPs)
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
    // DUMP MODE (Operator left stick click): vacuum with the roller BACKWARDS (X-take + auto-pass).
    private final VacuumCommand dumpCommand = new VacuumCommand(aim, shooter, index, shooterIntake,
        goober, gooba, groundIntakePivot, intakeRoller, aimCommand, true);
    public final PowerBudget powerBudget = new PowerBudget(drivetrain, aim,
        () -> driverController.getHID().getLeftStickButton());
    public final BackupShots backupShots = new BackupShots(drivetrain, localizer);
    // Defense mode (Driver Y): intake up, hood down, quick drive, X-lock, pin + tower warnings.
    public final DefenseMode defense = new DefenseMode(drivetrain, aim);
    // PRACTICE MATCH: random auto winner without an FMS + the match screen at http://10.56.69.2:5806
    public final PracticeMatch practiceMatch = new PracticeMatch(shooter, aim, () -> this.autoWinnerChooser.getSelected());

    // Not created on purpose: MariosEar (the 2 PhotonVision USB cameras), since the
    // Limelight is the only camera.

    public final DriverDashboard driverDashboard = new DriverDashboard();

    private final SendableChooser<Command> autoChooser;

    // State Toggles
    private boolean m_isShooterIdle = false;
    private static final String kStayPutAuto = "Shoot Preload (stay put)";
    private double lastAimRpm = 0; // auto-aim's last shooter speed (kept while RT is held)
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
        // DEFAULT (audit Oct 27): forgetting to pick an auto used to run "None" (zero points). Now
        // it shoots the preload from wherever the robot sits: it never drives, so it is safe from
        // any starting spot. Aims from the camera's position (tag-following if that's unknown).
        autoChooser.setDefaultOption(kStayPutAuto, Marcos.scoreUntilEmpty(aim, shooter, index, shooterIntake,
            goober, gooba, limelight, drivetrain).withName(kStayPutAuto));
        SmartDashboard.putData("Auto Mode", autoChooser);
        drivetrain.setKeepHeadingAtAutoStart(localizer::headingFromCamera, localizer::markHeadingFromPath);
        pitCheck.setMatchSetup(this::matchSetupProblems);
        practiceMatch.setSources(drivetrain, () -> operatorController.getHID().getRightTriggerAxis() > 0.5,
            this::selectedAutoName);

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
                    // (also Driver RB, Oct 28: holding Start takes the right thumb off the rotate stick)
                    if (driverController.getHID().getStartButton()
                            || (DriverStation.isTeleopEnabled() && driverController.getHID().getRightBumperButton())) {
                        speedMultiplier = buttonSpeedLimiter.getSelected();
                    }
                }

                // Stick dead zone FIRST (translation by stick magnitude, so diagonals stay true), rescaled
                // so just past it = just moving; then the cubic curve; then slew-rate limiting.
                double mag = Math.hypot(xInput, yInput);
                double magDb = MathUtil.applyDeadband(Math.min(1.0, mag), Constants.Operator.kStickDeadband);
                double xIn = mag > 1e-6 ? xInput / mag * magDb : 0.0;
                double yIn = mag > 1e-6 ? yInput / mag * magDb : 0.0;
                double rIn = MathUtil.applyDeadband(rInput, Constants.Operator.kStickDeadband);
                xInput = xIn; yInput = yIn; rInput = rIn;
                double cx = Math.signum(xInput) * Math.pow(Math.abs(xInput), 3);
                double cy = Math.signum(yInput) * Math.pow(Math.abs(yInput), 3);
                double cr = Math.signum(rInput) * Math.pow(Math.abs(rInput), 3);
                // SHOOTING ASSISTS (Oct 28, from 581/5000/1678/6328), while SCORING with RT held or the
                // robot firing (vacuum), teleop, robot position known, Driver left stick click NOT held
                // (full-power override): speed capped just under the 1.5 m/s fire limit, so the robot
                // keeps firing while the driver creeps instead of silently stopping. The cap is applied
                // BEFORE the slew limiters, so pressing or letting go of RT ramps the speed instead of
                // braking hard or lurching (bug hunt Oct 28). Passing keeps full speed (allowed to
                // 4 m/s). Switch "Slow Down While Shooting".
                var opHid = operatorController.getHID();
                boolean scoringNow = DriverStation.isTeleopEnabled() && aim.isActive() && !aim.passing()
                    && aim.poseUsable()
                    && (opHid.getRightTriggerAxis() > 0.5 || aim.isFiring())
                    && !driverController.getHID().getLeftStickButton()
                    && SmartDashboard.getBoolean(kSlowShootKey, true);
                if (scoringNow && !DefenseMode.active()) {
                    double capFrac = Constants.Aim.kMaxScoreSpeed * 0.9 / (maxSpeed * speedMultiplier);
                    double c = Math.hypot(cx, cy);
                    if (c > capFrac) { cx *= capFrac / c; cy *= capFrac / c; }
                }
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
                // Oct 28: only while a shot is wanted (auto-aim is on all teleop now, and a robot that
                // turns itself while the driver is just driving feels like it's fighting them).
                if (!defending && aim.isActive() && aim.shotWanted() && aim.poseUsable() && !aim.solution().reachable()
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

                // Sticks centered + nearly stopped + hub in reach while scoring = wheels lock in an X
                // (581, 6328), so a bump from a defender doesn't knock the shot off.
                double vx = scaledX * currentMaxSpeed, vy = scaledY * currentMaxSpeed;
                if (scoringNow) {
                    var st = drivetrain.getState().Speeds;
                    boolean centered = Math.hypot(xInput, yInput) < 1e-6 && Math.abs(rInput) < 1e-6;
                    if (centered && aim.poseUsable() && aim.solution().reachable()
                            && Math.hypot(st.vxMetersPerSecond, st.vyMetersPerSecond) < 0.3
                            && Math.abs(st.omegaRadiansPerSecond) < 0.3) {
                        return xLock;
                    }
                }

                // --- SWERVE DRIVE LOGIC ---
                if ("robot".equals(driveModeChooser.getSelected())) {
                    return robotCentricDrive
                        .withVelocityX(vx)
                        .withVelocityY(vy)
                        .withRotationalRate(rotationRate);
                }

                return fieldCentricDrive
                    .withVelocityX(vx)
                    .withVelocityY(vy)
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
    /**
     * TELEOP ONLY (audit Oct 27): PathPlannerAuto owns every subsystem its auto uses, so ANY button
     * that needs one of them (operator holding LT to pre-spin, RT, RB, a D-pad jog, Driver LB...)
     * cancelled the WHOLE rest of the auto. Every controller binding goes through this.
     */
    private final Trigger teleop = RobotModeTriggers.teleop();

    private Trigger tele(Trigger t) { return t.and(teleop); }

    private static final String kAimAtTeleopKey = "Auto-Aim On At Teleop Start";
    private static final String kSlowShootKey = "Slow Down While Shooting";

    /**
     * AUTO-AIM ON BY DEFAULT (Oct 28): at teleop start (and when defense is turned off with Y) the
     * robot turns auto-aim on by itself, so the operator's first job each match is gone. With aim
     * on but no shot wanted, only the TURRET tracks (the camera keeps seeing hub tags = better
     * position): the hood stays DOWN (trench-safe) and the robot never turns itself; RT, LT or
     * vacuum/dump bring the hood up and allow the dead-zone heading assist. Operator RB still turns
     * it off. Dashboard switch "Auto-Aim On At Teleop Start" (saved, ON).
     */
    private void autoAimOnIfWanted() {
        boolean on = SmartDashboard.getBoolean(kAimAtTeleopKey, true);
        if (on != Preferences.getBoolean(kAimAtTeleopKey, true)) Preferences.setBoolean(kAimAtTeleopKey, on);
        if (!on || DefenseMode.active() || aimCommand.isScheduled()) return;
        // Bug hunt Oct 28: with the position unknown (no auto run, no tags seen yet) auto-aim would
        // only follow a hub tag, so RT never fed and the turret swung by itself in the pit. Then it
        // stays OFF (RT works the old way) and the dashboard says why. A real match always starts
        // with an auto, which sets the position.
        if (!aim.poseUsable()) {
            SmartDashboard.putString("Auto-Aim At Teleop Start", "OFF: position unknown (RB turns it on)");
            return;
        }
        SmartDashboard.putString("Auto-Aim At Teleop Start", "ON");
        CommandScheduler.getInstance().schedule(aimCommand);
    }

    private void configureDriverBindings() {
        // LOCKED Left Bumper: reset gyro / seed field centric
        // Shoot from anywhere: this also tells the robot its heading matches the field, so press it
        // with the robot pointing straight AWAY from your driver station.
        // Oct 28: HOLD it half a second (a habit tap at the wrong heading used to wreck field-centric
        // driving and the aim); one short buzz = done.
        tele(driverController.leftBumper()).debounce(0.5).onTrue(drivetrain.runOnce(() -> {
            drivetrain.seedFieldCentric();
            localizer.markHeadingKnown();
            Haptics.on(Haptics.Pad.DRIVER);
        }));

        // LOCKED Right Trigger (hold): ground intake roller. Does nothing until the
        // roller is on CAN and kIntakeRollerEnabled = true.
        tele(driverController.rightTrigger()).whileTrue(intakeRoller.runCommand());

        // Left Trigger (hold): ground intake roller BACKWARDS, to spit balls out.
        // Added Oct 25 by the programming lead (was unused).
        tele(driverController.leftTrigger()).whileTrue(intakeRoller.reverseCommand());

        // LOCKED A (press): toggle ground intake DOWN / UP. Does nothing until
        // kDownPosition is set.
        tele(driverController.a()).onTrue(groundIntakePivot.toggleCommand());

        // LOCKED X: reserved (was the climb piston). Bind nothing.

        // Y (press): DEFENSE MODE toggle (added Oct 27). On: auto-aim and vacuum off, intake UP,
        // hood DOWN, quicker drive response, wheels lock in an X when the sticks are centered,
        // pin (G418) and tower (G420) warnings. See subsystems/DefenseMode.
        tele(driverController.y()).onTrue(defense.toggleCommand(() -> {
            vacuumCommand.cancel();
            dumpCommand.cancel();
            aimCommand.cancel();
            groundIntakePivot.stowIfDown();
            CommandScheduler.getInstance().schedule(gooba.stowCommand());
        }, this::autoAimOnIfWanted)); // defense OFF by Y: auto-aim back on (Oct 28)

        // LOCKED D-pad Up/Down (hold): climb up / down
        tele(driverController.povUp()).whileTrue(elevator.upCommand());
        tele(driverController.povDown()).whileTrue(elevator.downCommand());

        // LOCKED D-pad Left (press): shooter idle at 2000 RPM. Press again to turn it off.
        tele(driverController.povLeft()).onTrue(new InstantCommand(this::toggleShooterIdleMode));
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
        // RT resumes by itself after B (unjam) is let go (Oct 28: it used to stay dead until re-pressed).
        Trigger notUnjamming = operatorController.b().negate();
        tele(operatorController.rightTrigger()).and(notUnjamming).and(aimOn).whileTrue(
            new ShootCommand(aim, shooter, index, shooterIntake, goober, gooba, false));
        // Aim OFF: RT spins the wheel up by itself (shooter default) and feeds once it's at speed
        // (Oct 28: it used to feed balls into a stopped wheel unless LT was held first).
        tele(operatorController.rightTrigger()).and(notUnjamming).and(aimOn.negate()).whileTrue(
            new FeedShooterCommand(index, shooterIntake,
                // 75%: the fast preset (5400) is above what the wheel can HOLD (~4720 at 12 V, ~4300 at
                // 11 V, kV 0.15), so "90% of target" would never come and RT would never feed.
                () -> shooter.targetRpm() > 500 && shooter.getCurrentRpm() > 0.75 * shooter.targetRpm()));

        // VACUUM MODE (Back, press to toggle): intake + shoot hands-free. Intake down, roller in,
        // auto-aim on, fires by itself whenever ready. Scores from inside our alliance zone,
        // PASSES home from the neutral zone (follows the Start target mode). See VacuumCommand.
        // Operator RT/B and the jogs take over only while held; vacuum resumes after.
        // Vacuum and dump are two flavors of one hands-free mode: turning one on ends the other.
        tele(operatorController.back()).onTrue(Commands.runOnce(() -> {
            if (vacuumCommand.isScheduled()) vacuumCommand.cancel();
            else { dumpCommand.cancel(); CommandScheduler.getInstance().schedule(vacuumCommand); }
        }).withName("Operator Back: vacuum toggle"));
        // DUMP MODE (left stick CLICK, press to toggle): the trench X-take with a full hopper, plus
        // auto-passing. Roller BACKWARDS (balls out the front), auto-aim on, fires by itself when
        // ready (passes from the neutral zone; pauses in the trench, hood down; in our zone scores
        // only into an open hub). Lower the intake first (Driver A). Ends like vacuum: Operator RB,
        // Y, Back (switches to vacuum), Driver Y (defense). Driver LT/RT still take the roller.
        // Bug hunt Oct 28: the intake jog is on the same stick, and a hard push often clicks it, so
        // the click only counts with the stick near center (not pushed for the last 0.3 s).
        Trigger leftStickCentered = new Trigger(() -> Math.abs(operatorController.getLeftY()) < 0.3
            && Math.abs(operatorController.getLeftX()) < 0.3).debounce(0.3);
        tele(operatorController.leftStick().and(leftStickCentered)).onTrue(Commands.runOnce(() -> {
            if (dumpCommand.isScheduled()) dumpCommand.cancel();
            else { vacuumCommand.cancel(); CommandScheduler.getInstance().schedule(dumpCommand); }
        }).withName("Operator L3: dump toggle"));
        // Y (manual hood) means "I'm taking over": vacuum mode turns off. RB does too (below).
        tele(operatorController.y()).onTrue(Commands.runOnce(() -> { vacuumCommand.cancel(); dumpCommand.cancel(); }));

        // SHOT TARGET MODE (Start, press): AUTO -> SCORE only -> PASS only -> AUTO. Dashboard
        // "Shot Target" shows it. AUTO = score inside our alliance zone, pass from past it.
        tele(operatorController.start()).onTrue(aim.cycleModeCommand());

        // LOCKED Left Trigger (hold): shooter windup. Auto-aim ON: to the shot table's speed for
        // the distance (the shooter's default command does it, see configureSystemDefaults).
        // Auto-aim OFF: the fast preset, as before.
        tele(operatorController.leftTrigger()).and(aimOn.negate())
            .whileTrue(new RunShooterCommand(shooter, Constants.Shooter.kFastTargetRPM));

        // LOCKED Right Bumper (press): toggle Limelight auto-aim. First press starts
        // turret tracking, next press stops it. With no tag in view, the turret sweeps
        // to look for one. Using the turret jog also stops the auto-aim. Does nothing
        // until the turret's far end is set ("Set Turret Far End" dashboard button).
        // The hood (AutoGooba) follows the tag too. Using the turret jog, the hood jog or Y
        // stops the whole auto-aim (turret and hood).
        // SHOOT FROM ANYWHERE (Oct 26): aims from the robot's field position (falls back to the
        // hub tag when the position isn't known). See AimCommand.
        // One press handler (audit Oct 27): with vacuum on, RB ends vacuum AND auto-aim (before, the
        // vacuum cancel ran first, then the toggle saw aim off and turned it right back on).
        tele(operatorController.rightBumper()).onTrue(Commands.runOnce(() -> {
            if (vacuumCommand.isScheduled() || dumpCommand.isScheduled()) {
                vacuumCommand.cancel();
                dumpCommand.cancel();
                aimCommand.cancel();
            } else if (aimCommand.isScheduled()) {
                aimCommand.cancel();
            } else {
                CommandScheduler.getInstance().schedule(aimCommand);
            }
        }).withName("Operator RB: auto-aim toggle"));

        // BACKUP SHOTS (camera-proof): park on the spot, press its button, then hold RT.
        // A = Midshot (between hub and climb), X = Left trench, LB = Right trench (left/right as
        // seen from our driver station). See subsystems/BackupShots.
        tele(operatorController.a()).debounce(0.5).onTrue(backupShots.useCommand(BackupShots.Spot.MIDSHOT, aimCommand));
        // The trench spots sit right next to a trench: these also turn on the trench-shot override
        // (hood may go up while the robot stays put).
        tele(operatorController.x()).debounce(0.5).onTrue(backupShots.useCommand(BackupShots.Spot.LEFT_TRENCH, aimCommand)
            .andThen(aim.trenchOverrideCommand()));
        tele(operatorController.leftBumper()).debounce(0.5).onTrue(backupShots.useCommand(BackupShots.Spot.RIGHT_TRENCH, aimCommand)
            .andThen(aim.trenchOverrideCommand()));

        // TRENCH GUARD (automatic): near a trench the hood goes all the way DOWN (the Limelight on
        // the hood is taller than the trench bar with the hood up). Right stick CLICK (R3) = trench
        // shot: hood may go up while the robot stays put next to the trench; moving ends it.
        tele(operatorController.rightStick()).onTrue(aim.trenchOverrideCommand());

        // LOCKED Y (press): toggle the hood between stowed and deployed (the drive team's
        // map: "Toggle Gooba Hood"). Back now that the hood motor (18) is.
        tele(operatorController.y()).onTrue(gooba.toggleCommand());

        // D-pad Up/Down (hold): jog the hood slowly (programming lead, Oct 25; same as last
        // season). Write down "Hood Position (rot)" for good shots.
        tele(operatorController.povUp()).whileTrue(new ManualGoobaCommand(gooba, true));
        tele(operatorController.povDown()).whileTrue(new ManualGoobaCommand(gooba, false));

        // Left Stick up/down (hold): jog the ground intake slowly. Manual backup for when the
        // Driver A toggle can't do it (ignores the intake's soft limits while held). Was the D-pad.
        tele(new Trigger(() -> operatorController.getLeftY() < -0.5 && !operatorController.getHID().getLeftStickButton())).whileTrue(groundIntakePivot.jogUpCommand());
        tele(new Trigger(() -> operatorController.getLeftY() > 0.5 && !operatorController.getHID().getLeftStickButton())).whileTrue(groundIntakePivot.jogDownCommand());

        // OPEN D-pad Left/Right (hold): manual turret jog. Right = turn right (away from HOME).
        // The diagonals count too: a D-pad often reads Right as Up-Right or Down-Right for a moment,
        // and with plain Left/Right that stopped and restarted the turret (it stuttered).
        Trigger turretJogLeft = tele(operatorController.povLeft())
            .or(tele(operatorController.povUpLeft())).or(tele(operatorController.povDownLeft()));
        Trigger turretJogRight = tele(operatorController.povRight())
            .or(tele(operatorController.povUpRight())).or(tele(operatorController.povDownRight()));
        turretJogLeft.whileTrue(new ManualTurretCommand(goober, -Constants.Turret.kManualJogSpeed));
        turretJogRight.whileTrue(new ManualTurretCommand(goober, Constants.Turret.kManualJogSpeed));

        // Not on the map, kept on purpose: B (hold) runs the Spindex backwards to clear a jam
        tele(operatorController.b()).whileTrue(new StartEndCommand(
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

        Preferences.initBoolean(kSlowShootKey, true);
        SmartDashboard.putBoolean(kSlowShootKey, Preferences.getBoolean(kSlowShootKey, true));
        new Trigger(() -> SmartDashboard.getBoolean(kSlowShootKey, true) != Preferences.getBoolean(kSlowShootKey, true))
            .onTrue(Commands.runOnce(() -> Preferences.setBoolean(kSlowShootKey, SmartDashboard.getBoolean(kSlowShootKey, true)))
                .ignoringDisable(true));
        Preferences.initBoolean(kAimAtTeleopKey, true);
        SmartDashboard.putBoolean(kAimAtTeleopKey, Preferences.getBoolean(kAimAtTeleopKey, true));
        // Saved the moment they're flipped (were saved only at teleop start, so Pit Check read stale values).
        for (String key : new String[] {kAimAtTeleopKey, "Practice: Random Auto Winner"}) {
            new Trigger(() -> SmartDashboard.getBoolean(key, true) != Preferences.getBoolean(key, true))
                .onTrue(Commands.runOnce(() -> Preferences.setBoolean(key, SmartDashboard.getBoolean(key, true)))
                    .ignoringDisable(true));
        }
        // Published at boot so the Drive Team tab never shows grey "missing" widgets.
        SmartDashboard.putBoolean("Vacuum Mode", false);
        SmartDashboard.putBoolean("Dump Mode", false);
        SmartDashboard.putBoolean("Shooter Idle Active", false);
        SmartDashboard.putString("Auto-Aim At Teleop Start", "waiting for teleop");
        RobotModeTriggers.teleop().onTrue(Commands.runOnce(this::autoAimOnIfWanted).withName("Teleop start: auto-aim on"));

        shooter.setDefaultCommand(new RunCommand(() -> {
            boolean wantsShot = operatorController.getHID().getRightTriggerAxis() > 0.5
                || operatorController.getHID().getLeftTriggerAxis() > 0.5
                || aim.vacuum();   // vacuum mode: keep the wheel up even while an unjam has the feeders
            boolean rtHeld = operatorController.getHID().getRightTriggerAxis() > 0.5;
            if (aim.isActive() && wantsShot) {
                shooter.runAtRPM(aim.targetRpm()); // auto-aim: table speed while LT or RT is held
                lastAimRpm = aim.targetRpm();
            } else if (rtHeld && lastAimRpm > 0 && DriverStation.isTeleopEnabled()) {
                // Auto-aim ended mid-volley (a jog, Y, defense) while RT still feeds: keep the wheel at
                // its last speed instead of stopping it under the balls (audit Oct 27).
                shooter.runAtRPM(lastAimRpm);
            } else if (rtHeld && DriverStation.isTeleopEnabled()) {
                // Auto-aim OFF and RT held: spin up to the fast preset (the feeders wait for it).
                shooter.runAtRPM(Constants.Shooter.kFastTargetRPM);
            } else if (m_isShooterIdle) {
                shooter.runAtRPM(Constants.Shooter.kIdleRPM);
            } else {
                shooter.stop();
                if (!rtHeld) lastAimRpm = 0;
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

    /**
     * A match is starting. Leftovers from practice go back to normal (target mode, defense), and a
     * PathPlanner auto's path sets the field position. The stay-put auto sets nothing: it trusts
     * only what the camera already found.
     */
    public void onAutoStart() {
        aim.resetTargetMode();
        DefenseMode.reset();
        if (m_isShooterIdle) toggleShooterIdleMode(); // idle left on from practice: off
        if (autoChooser.getSelected() instanceof com.pathplanner.lib.commands.PathPlannerAuto) {
            localizer.markPoseSetByAuto();
        }
    }

    /** Name of the selected auto ("None" = does nothing). */
    public String selectedAutoName() {
        Command c = autoChooser.getSelected();
        return c == null ? "None" : c.getName();
    }

    /**
     * MATCH SETUP for Pit Check: things left over from practice or tuning that would cost a match.
     */
    public java.util.List<String> matchSetupProblems() {
        var p = new java.util.ArrayList<String>();
        Command c = autoChooser.getSelected();
        if (c == null || c.getName().equals("InstantCommand") || c.getName().equals("None"))
            p.add("NO AUTO selected (does nothing)");
        if (autoWinnerChooser.getSelected() != null && autoWinnerChooser.getSelected() != HubShiftTracker.AutoWinner.FMS)
            p.add("Auto Winner is not FMS: set it back");
        if (aim.currentMode() != frc.robot.aiming.TargetPicker.Mode.AUTO) p.add("Shot Target mode is " + aim.currentMode() + ", not AUTO");
        if (aim.tuningMode()) p.add("Shot Tuning Mode is ON: turn it off");
        if (!SmartDashboard.getBoolean("Vision Pose ON", true)) p.add("Vision Pose ON is off: turn it on");
        if (gooba.limitsOff()) p.add("Hood limits are OFF (measure switch)");
        if (elevator.limitsOff()) p.add("Elevator limits are OFF (recovery switch)");
        if (DefenseMode.active()) p.add("Defense mode is ON");
        if (!"field".equals(driveModeChooser.getSelected())) p.add("Drive Mode is Robot Centric");
        Double lim = globalSpeedLimiter.getSelected();
        if (lim != null && lim < 1.0) p.add("Global Speed Limit is " + Math.round(lim * 100) + "%");
        if (m_isShooterIdle) p.add("Shooter idle is ON");
        // Saved switches left off their match settings after practice/tuning.
        String[][] saved = {{"Hold Fire: Hub Closed", "true"}, {"Auto Agitate", "true"}, {"Agitate Drive Shimmy", "false"},
            {"Power Budget ON", "true"}, {"Vision Fixes Heading", "true"}, {"Climb Shot Table", "true"},
            {"Auto: Turret Watches Hub", "true"}, {kAimAtTeleopKey, "true"}, {kSlowShootKey, "true"}};
        for (String[] k : saved) {
            boolean want = Boolean.parseBoolean(k[1]);
            if (Preferences.containsKey(k[0]) && Preferences.getBoolean(k[0], want) != want)
                p.add("\"" + k[0] + "\" is " + (want ? "OFF" : "ON") + " (match setting: " + (want ? "ON" : "OFF") + ")");
        }
        if (!DriverStation.isJoystickConnected(0)) p.add("no DRIVER controller in slot 0");
        if (!DriverStation.isJoystickConnected(1)) p.add("no OPERATOR controller in slot 1");
        return p;
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

    /**
     * AUTO START CHECK (while disabled): is the robot where the selected auto starts? The camera
     * gives the position; the selected PathPlanner auto gives its start (flipped for red). Says
     * which way to nudge the robot, in the drive team's words, so a misplaced robot doesn't run the
     * whole auto shifted. Also catches "no auto selected" (the chooser's default does nothing).
     */
    private void updateAutoStartCheck() {
        Command sel = autoChooser.getSelected();
        String text;
        boolean ok = false;
        if (sel != null && kStayPutAuto.equals(sel.getName())) {
            ok = localizer.poseTrusted();
            text = ok ? "stay-put auto: shoots the preload from here (position known)"
                : "stay-put auto: position unknown, will follow the hub tag (point the turret at the hub)";
        } else if (!(sel instanceof com.pathplanner.lib.commands.PathPlannerAuto auto)) {
            text = "NO AUTO SELECTED: pick one in \"Auto Mode\"";
        } else {
            var start = auto.getStartingPose();
            boolean red = DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Red).orElse(false);
            if (start == null) {
                text = "auto has no start pose";
            } else if (!localizer.poseTrusted()) {
                if (red) start = com.pathplanner.lib.util.FlippingUtil.flipFieldPose(start);
                text = "can't check: position unknown (camera needs to see 2 tags)";
            } else {
                if (red) start = com.pathplanner.lib.util.FlippingUtil.flipFieldPose(start);
                var pose = drivetrain.getState().Pose;
                double dx = start.getX() - pose.getX(), dy = start.getY() - pose.getY();
                // Driver's words: "away" = away from our driver station, "left" = driver's left.
                double away = red ? -dx : dx, left = red ? -dy : dy;
                double turn = start.getRotation().minus(pose.getRotation()).getDegrees();
                double dist = Math.hypot(dx, dy);
                ok = dist < 0.10 && Math.abs(turn) < 5.0;
                text = ok ? String.format(java.util.Locale.US, "OK (%.0f in, %.0f deg off)", dist / 0.0254, turn)
                    : String.format(java.util.Locale.US, "MOVE %.0f in %s, %.0f in %s, turn %.0f deg %s",
                        Math.abs(away) / 0.0254, away >= 0 ? "AWAY from DS" : "TOWARD DS",
                        Math.abs(left) / 0.0254, left >= 0 ? "LEFT" : "RIGHT",
                        Math.abs(turn), turn >= 0 ? "left (CCW)" : "right (CW)");
            }
        }
        SmartDashboard.putString("Auto Start", text);
        SmartDashboard.putBoolean("Auto Start OK", ok);
    }

    public void updateDashboard() {
        if (DriverStation.isDisabled()) updateAutoStartCheck();
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
        double pattern = Haptics.level(Haptics.Pad.OPERATOR);
        operatorController.getHID().setRumble(RumbleType.kBothRumble, pattern >= 0 ? pattern : ready ? 0.4 : 0.0);
        var op = operatorController.getHID();
        aim.setShotWanted(!DriverStation.isTeleopEnabled() || op.getRightTriggerAxis() > 0.5
            || op.getLeftTriggerAxis() > 0.5 || aim.vacuum() || aim.isFiring());
        SmartDashboard.putBoolean("Ready To Fire", ready);
        // Which Xbox controller is which: slot 0 = DRIVER, slot 1 = OPERATOR in the
        // Driver Station USB tab. Driver A / Right Trigger / D-pad only work from slot 0.
        SmartDashboard.putBoolean("Driver Controller (slot 0)", DriverStation.isJoystickConnected(0));
        SmartDashboard.putBoolean("Operator Controller (slot 1)", DriverStation.isJoystickConnected(1));
        SmartDashboard.putBoolean("HOTAS Connected", hotas.isConnected());
        SmartDashboard.putBoolean("HOTAS Throttle Connected", hotas.isThrottleConnected());
    }
}
