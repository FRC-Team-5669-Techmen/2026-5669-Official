// Copyright (c) FIRST and other WPILib contributors.
// SigmaAura-est was here...

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.controls.HotasX56;
import frc.robot.generated.TunerConstants;

// Subsystems
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Goober;
import frc.robot.subsystems.GroundIntakePivotSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.MariosEar;
import frc.robot.subsystems.ShooterIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// Commands
import frc.robot.commands.FeedShooterCommand;
import frc.robot.commands.GooberAlign;
import frc.robot.commands.ManualTurretCommand;
import frc.robot.commands.RunShooterCommand;

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
    public final MariosEar mariosEar = new MariosEar(limelight);
    public final GroundIntakePivotSubsystem groundIntakePivot = new GroundIntakePivotSubsystem();
    public final ElevatorSubsystem elevator = new ElevatorSubsystem();  // climber, CAN 19
    public final IntakeRollerSubsystem intakeRoller = new IntakeRollerSubsystem(); // placeholder until on CAN

    // Not created on purpose: the hood (GoobaSubsystem), since there's no hood motor.

    public final DriverDashboard driverDashboard = new DriverDashboard();

    private final SendableChooser<Command> autoChooser;

    // State Toggles
    private boolean m_isShooterIdle = false;

    public RobotContainer() {
        // We intentionally configure more HIDs (Xbox x2 + HOTAS x2) than are usually
        // plugged in, so don't spam the DS console about the empty ports.
        DriverStation.silenceJoystickConnectionWarning(true);

        Marcos.registerNamedCommands(
            shooter, index, shooterIntake, goober, limelight, mariosEar
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
                double scaledX = xLimiter.calculate(Math.signum(xInput) * Math.pow(Math.abs(xInput), 3));
                double scaledY = yLimiter.calculate(Math.signum(yInput) * Math.pow(Math.abs(yInput), 3));
                double scaledRot = rotLimiter.calculate(Math.signum(rInput) * Math.pow(Math.abs(rInput), 3));

                double currentMaxSpeed = maxSpeed * speedMultiplier;
                double currentMaxAngularRate = maxAngularRate * speedMultiplier;

                // --- SWERVE DRIVE LOGIC ---
                if ("robot".equals(driveModeChooser.getSelected())) {
                    return robotCentricDrive
                        .withVelocityX(scaledX * currentMaxSpeed)
                        .withVelocityY(scaledY * currentMaxSpeed)
                        .withRotationalRate(scaledRot * currentMaxAngularRate);
                }

                return fieldCentricDrive
                    .withVelocityX(scaledX * currentMaxSpeed)
                    .withVelocityY(scaledY * currentMaxSpeed)
                    .withRotationalRate(scaledRot * currentMaxAngularRate);
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
        driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // LOCKED Right Trigger (hold): ground intake roller. Does nothing until the
        // roller is on CAN and kIntakeRollerEnabled = true.
        driverController.rightTrigger().whileTrue(intakeRoller.runCommand());

        // LOCKED A (press): toggle ground intake DOWN / UP. Does nothing until
        // kDownPosition is set.
        driverController.a().onTrue(groundIntakePivot.toggleCommand());

        // LOCKED X: reserved (was the climb piston). Bind nothing.

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
        // LOCKED Right Trigger (hold): shoot (Spindex + uptake)
        operatorController.rightTrigger().whileTrue(new FeedShooterCommand(index, shooterIntake));

        // LOCKED Left Trigger (hold): shooter windup
        operatorController.leftTrigger().whileTrue(new RunShooterCommand(shooter, Constants.Shooter.kFastTargetRPM));

        // LOCKED Right Bumper (press): toggle Limelight auto-aim. First press starts
        // turret tracking, next press stops it. With no tag in view, the turret sweeps
        // to look for one. Using the turret jog also stops the auto-aim.
        operatorController.rightBumper().toggleOnTrue(new GooberAlign(limelight, goober));

        // LOCKED Y: reserved (was the "Gooba" hood). Bind nothing.

        // OPEN D-pad Up/Down (hold): jog the ground intake slowly. Used to measure the
        // DOWN position, then kept as a manual backup.
        operatorController.povUp().whileTrue(groundIntakePivot.jogUpCommand());
        operatorController.povDown().whileTrue(groundIntakePivot.jogDownCommand());

        // OPEN D-pad Left/Right (hold): manual turret jog
        operatorController.povLeft().whileTrue(new ManualTurretCommand(goober, -Constants.Turret.kManualJogSpeed));
        operatorController.povRight().whileTrue(new ManualTurretCommand(goober, Constants.Turret.kManualJogSpeed));

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

        // Dashboard button for when the code restarts with the intake somewhere other than UP.
        SmartDashboard.putData("Zero Intake (intake is UP)", groundIntakePivot.zeroCommand());

        // Elevator: zero button (elevator all the way down), and a recovery switch that turns
        // the soft limits off in case it was powered on with the elevator up.
        SmartDashboard.putData("Zero Elevator (elevator is DOWN)", elevator.zeroCommand());
        SmartDashboard.putData("Elevator Limits OFF (recovery)", elevator.limitsOffCommand());

        shooter.setDefaultCommand(new RunCommand(() -> {
            if (m_isShooterIdle) {
                shooter.runAtRPM(Constants.Shooter.kIdleRPM);
            } else {
                shooter.stop();
            }
        }, shooter));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void toggleShooterIdleMode() {
        m_isShooterIdle = !m_isShooterIdle;
        SmartDashboard.putBoolean("Shooter Idle Active", m_isShooterIdle);
        System.out.println("Shooter Idle State Toggled: " + m_isShooterIdle);
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
        // Which Xbox controller is which: slot 0 = DRIVER, slot 1 = OPERATOR in the
        // Driver Station USB tab. Driver A / Right Trigger / D-pad only work from slot 0.
        SmartDashboard.putBoolean("Driver Controller (slot 0)", DriverStation.isJoystickConnected(0));
        SmartDashboard.putBoolean("Operator Controller (slot 1)", DriverStation.isJoystickConnected(1));
        SmartDashboard.putBoolean("HOTAS Connected", hotas.isConnected());
        SmartDashboard.putBoolean("HOTAS Throttle Connected", hotas.isThrottleConnected());
    }
}
