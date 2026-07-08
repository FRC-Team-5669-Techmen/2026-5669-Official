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
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GoobaSubsystem;
import frc.robot.subsystems.Goober;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.MariosEar;
import frc.robot.subsystems.ShooterIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// Commands
import frc.robot.commands.AutoGooba;
import frc.robot.commands.FeedShooterCommand;
import frc.robot.commands.GooberAlign;
import frc.robot.commands.ManualGoobaCommand;
import frc.robot.commands.ManualTurretCommand;
import frc.robot.commands.RunClimbMotorCommand;
import frc.robot.commands.RunGroundIntakeCommand;
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
    public final GoobaSubsystem gooba = new GoobaSubsystem();
    public final Goober goober = new Goober();
    public final LimelightSubsystem limelight = new LimelightSubsystem();
    public final MariosEar mariosEar = new MariosEar(limelight);
    public final GroundIntakeSubsystem groundIntake = new GroundIntakeSubsystem();
    public final ClimbSubsystem climb = new ClimbSubsystem();

    public final DriverDashboard driverDashboard = new DriverDashboard();

    private final SendableChooser<Command> autoChooser;

    // State Toggles
    private boolean m_isShooterIdle = false;
    private boolean m_isSimulatedTankDrive = false; // Swerve acts like a traditional tank drive (no strafe)

    public RobotContainer() {
        // We intentionally configure more HIDs (Xbox x2 + HOTAS x2) than are usually
        // plugged in, so don't spam the DS console about the empty ports.
        DriverStation.silenceJoystickConnectionWarning(true);

        Marcos.registerNamedCommands(
            shooter, index, shooterIntake, gooba, goober, limelight, mariosEar, groundIntake
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

                // --- TANK DRIVE / NO-STRAFE LOGIC ---
                if (m_isSimulatedTankDrive) {
                    // Robot-centric with strafing disabled: stick Y = throttle, rotation = steering
                    return robotCentricDrive
                        .withVelocityX(scaledX * currentMaxSpeed)
                        .withVelocityY(0.0)
                        .withRotationalRate(scaledRot * currentMaxAngularRate);
                }

                // --- NORMAL SWERVE DRIVE LOGIC ---
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
    private void configureDriverBindings() {
        driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Right Trigger: Normal Intake
        driverController.rightTrigger().whileTrue(new RunGroundIntakeCommand(groundIntake));

        // Left Trigger: Reverse Intake (Spit out)
        driverController.leftTrigger().whileTrue(reverseGroundIntakeCommand());

        // X and A are unbound (they used to toggle the removed pneumatic pistons)

        driverController.povUp().whileTrue(new RunClimbMotorCommand(climb, Constants.Climb.kClimbSpeed));
        driverController.povDown().whileTrue(new RunClimbMotorCommand(climb, -Constants.Climb.kClimbSpeed));

        driverController.povLeft().onTrue(new InstantCommand(this::toggleShooterIdleMode));
        driverController.povRight().onTrue(new InstantCommand(this::toggleTankDriveMode));
    }

    // ==========================================
    // --- OPERATOR CONTROLLER (PORT 1 - XBOX) ---
    // ==========================================
    private void configureOperatorBindings() {
        operatorController.leftTrigger().whileTrue(new RunShooterCommand(shooter, Constants.Shooter.kFastTargetRPM));
        operatorController.rightTrigger().whileTrue(new FeedShooterCommand(index, shooterIntake));

        // 'B' Button independently rewinds the Index Subsystem ONLY
        operatorController.b().whileTrue(new StartEndCommand(
            () -> index.run(Constants.Index.kReverseSpeed),
            index::stop,
            index
        ));

        // 'Y' toggles the hood between stowed and deployed
        operatorController.y().onTrue(new InstantCommand(() -> {
            if (Math.abs(gooba.getPosition()) > 1.0) {
                gooba.setPosition(Constants.Gooba.kPositionStowed);
            } else {
                gooba.setPosition(Constants.Gooba.kPositionDeployed);
            }
        }, gooba));

        // Left Bumper: track the hub — turret alignment + auto hood angle together
        operatorController.leftBumper().whileTrue(
            new GooberAlign(limelight, goober).alongWith(new AutoGooba(gooba, limelight)));

        operatorController.povUp().whileTrue(new ManualGoobaCommand(gooba, false));  // jog hood toward stow
        operatorController.povDown().whileTrue(new ManualGoobaCommand(gooba, true)); // jog hood toward deploy

        operatorController.povLeft().whileTrue(new ManualTurretCommand(goober, -Constants.Turret.kManualJogSpeed));
        operatorController.povRight().whileTrue(new ManualTurretCommand(goober, Constants.Turret.kManualJogSpeed));
    }

    // ==========================================
    // --- HOTAS (PORTS 2 & 3 - X56 STICK + THROTTLE) ---
    // ==========================================
    // Mirrors the driver Xbox layout so either input device can drive the robot.
    // Bindings are gated on the stick being plugged in; see docs/X56-HOTAS-SETUP.md.
    private void configureHotasBindings() {
        Trigger hotasConnected = new Trigger(hotas::isConnected);

        // Trigger finger: ground intake in, thumb (A): spit out
        hotas.trigger().and(hotasConnected).whileTrue(new RunGroundIntakeCommand(groundIntake));
        hotas.buttonA().and(hotasConnected).whileTrue(reverseGroundIntakeCommand());

        // B and C are unbound (they used to toggle the removed pneumatic pistons)

        // D: tank-drive toggle, Pinkie lever: re-zero field-centric heading
        hotas.buttonD().and(hotasConnected).onTrue(new InstantCommand(this::toggleTankDriveMode));
        hotas.pinkie().and(hotasConnected).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Main hat up/down: climb motor
        hotas.povUp().and(hotasConnected).whileTrue(new RunClimbMotorCommand(climb, Constants.Climb.kClimbSpeed));
        hotas.povDown().and(hotasConnected).whileTrue(new RunClimbMotorCommand(climb, -Constants.Climb.kClimbSpeed));
    }

    // ==========================================
    // --- SYSTEM DEFAULTS ---
    // ==========================================
    private void configureSystemDefaults() {
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        shooter.setDefaultCommand(new RunCommand(() -> {
            if (m_isShooterIdle) {
                shooter.runAtRPM(Constants.Shooter.kIdleRPM);
            } else {
                shooter.stop();
            }
        }, shooter));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    /** Runs the ground intake backwards to spit fuel out, stopping on release. */
    private Command reverseGroundIntakeCommand() {
        return new StartEndCommand(
            () -> groundIntake.runIntake(Constants.GroundIntake.kReverseSpeed),
            groundIntake::stop,
            groundIntake
        );
    }

    private void toggleShooterIdleMode() {
        m_isShooterIdle = !m_isShooterIdle;
        SmartDashboard.putBoolean("Shooter Idle Active", m_isShooterIdle);
        System.out.println("Shooter Idle State Toggled: " + m_isShooterIdle);
    }

    private void toggleTankDriveMode() {
        m_isSimulatedTankDrive = !m_isSimulatedTankDrive;
        SmartDashboard.putBoolean("Tank Drive Active", m_isSimulatedTankDrive);
        System.out.println("Tank Drive State Toggled: " + m_isSimulatedTankDrive);
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
        SmartDashboard.putBoolean("HOTAS Connected", hotas.isConnected());
        SmartDashboard.putBoolean("HOTAS Throttle Connected", hotas.isThrottleConnected());
    }
}
