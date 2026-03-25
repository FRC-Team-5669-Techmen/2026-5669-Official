// Copyright (c) FIRST and other WPILib contributors.
// SigmaAura-est was here...

package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.generated.TunerConstants;
import frc.robot.Constants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Subsystems
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterIntakeSubsystem;
import frc.robot.subsystems.GoobaSubsystem;
import frc.robot.subsystems.Goober;
import frc.robot.subsystems.MariosEar;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.ClimbSubsystem; 

// Commands
//Hashtg
import frc.robot.commands.RunShooterCommand;
import frc.robot.commands.FuelHandlingCommand;
import frc.robot.commands.FeedShooterCommand;
import frc.robot.commands.GooberAlign;
import frc.robot.commands.TogglePneumaticCommand;
import frc.robot.commands.ManualGoobaCommand;
import frc.robot.commands.ManualTurretCommand;
import frc.robot.commands.RunGroundIntakeCommand;
import frc.robot.commands.RunClimbMotorCommand;
import frc.robot.commands.AutoGooba;

public class RobotContainer {

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = Constants.Operator.kMaxAngularRate.in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * Constants.Operator.kDeadband)
            .withRotationalDeadband(MaxAngularRate * Constants.Operator.kRotationalDeadband)
            .withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * Constants.Operator.kDeadband)
            .withRotationalDeadband(MaxAngularRate * Constants.Operator.kRotationalDeadband)
            .withDriveRequestType(DriveRequestType.Velocity);

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(3.0);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3.0);

    private final SendableChooser<String> driveModeChooser = new SendableChooser<>();
    private final SendableChooser<Double> globalSpeedLimiter = new SendableChooser<>();
    private final SendableChooser<Double> buttonSpeedLimiter = new SendableChooser<>();
    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandXboxController driverController = new CommandXboxController(Constants.Operator.kDriverControllerPort);
    public final CommandXboxController operator = new CommandXboxController(Constants.Operator.kOperatorControllerPort);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final ShooterSubsystem shooter = new ShooterSubsystem();
    public final IndexSubsystem index = new IndexSubsystem();
    public final ShooterIntakeSubsystem shooterIntake = new ShooterIntakeSubsystem();
    public final GoobaSubsystem gooba = new GoobaSubsystem();
    public final Goober goober = new Goober();
    public final LimelightSubsystem rizz = new LimelightSubsystem();
    public final MariosEar brick = new MariosEar(rizz);
    public final GroundIntakeSubsystem groundIntake = new GroundIntakeSubsystem();
    public final ClimbSubsystem climb = new ClimbSubsystem();
/* public final PneumaticSubsystem ClimbPiston = new PneumaticSubsystem(
        Constants.Pneumatics.kPcmId, 
        Constants.Pneumatics.kSol1Forward, 
        Constants.Pneumatics.kSol1Reverse);
        */
    public final PneumaticSubsystem ClimbPiston = new PneumaticSubsystem(
        Constants.Pneumatics.kPcmId, 0);
    public final PneumaticSubsystem DoubleIntake = new PneumaticSubsystem(
        Constants.Pneumatics.kPcmId, 
        Constants.Pneumatics.kSol2Forward, 
        Constants.Pneumatics.kSol2Reverse);
   

    private final SendableChooser<Command> autoChooser;

    // State Toggles
    // UPDATED: Now set to FALSE by default so it doesn't drain battery on startup!
    private boolean m_continuousTurretAim = false; 
    private boolean m_isShooterIdle = false; // Controls if shooter maintains idle speed

    public RobotContainer() {
        Marcos.registerNamedCommands(
            shooter, index, shooterIntake, gooba, goober, rizz, brick, groundIntake, DoubleIntake, ClimbPiston
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

        configureBindings();
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // ==========================================
        // --- DRIVER CONTROLLER (PORT 0) ---
        // ==========================================

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double xInput = -driverController.getLeftY();
                double yInput = -driverController.getLeftX();
                double rInput = -driverController.getRightX();

                double scaledX = xLimiter.calculate(Math.signum(xInput) * Math.pow(Math.abs(xInput), 3));
                double scaledY = yLimiter.calculate(Math.signum(yInput) * Math.pow(Math.abs(yInput), 3));
                double scaledRot = rotLimiter.calculate(Math.signum(rInput) * Math.pow(Math.abs(rInput), 3));

                double speedMultiplier = globalSpeedLimiter.getSelected();
                if (driverController.getHID().getStartButton()) {
                    speedMultiplier = buttonSpeedLimiter.getSelected();
                }

                double currentMaxSpeed = MaxSpeed * speedMultiplier;
                double currentMaxAngularRate = MaxAngularRate * speedMultiplier;

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

        driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        driverController.rightTrigger().whileTrue(new RunGroundIntakeCommand(groundIntake));

        driverController.x().onTrue(new TogglePneumaticCommand(DoubleIntake));
        driverController.a().onTrue(new TogglePneumaticCommand(ClimbPiston));

        //driverController.a().whileTrue(new TogglePneumaticCommand(ClimbPiston));
        driverController.povUp().whileTrue(new RunClimbMotorCommand(climb, Constants.Climb.kClimbSpeed));
        driverController.povDown().whileTrue(new RunClimbMotorCommand(climb, -Constants.Climb.kClimbSpeed));

        // Driver D-Pad Left toggles the Idle Shooter Mode
        driverController.povLeft().onTrue(new InstantCommand(() -> {
            m_isShooterIdle = !m_isShooterIdle;
            System.out.println("Shooter Idle State Toggled: " + m_isShooterIdle);
        }));

        // ==========================================
        // --- OPERATOR CONTROLLER (PORT 1) ---
        // ==========================================

        operator.leftTrigger().whileTrue(new RunShooterCommand(shooter, Constants.Shooter.kfastTargetRPM));
        operator.rightTrigger().whileTrue(new FeedShooterCommand(index, shooterIntake));
        
        // 'B' Button now independently rewinds the Index Subsystem ONLY
        operator.b().whileTrue(new StartEndCommand(
            () -> index.run(Constants.Index.kReverseSpeed), 
            () -> index.stop(), 
            index
        ));

        operator.y().onTrue(new InstantCommand(() -> {
            if (Math.abs(gooba.getPosition()) > 1.0) {
                gooba.setPosition(Constants.Gooba.kPositionStowed);
            } else {
                gooba.setPosition(Constants.Gooba.kPositionDeployed);
            }
        }, gooba));

        operator.rightBumper().onTrue(new InstantCommand(() -> {
            m_continuousTurretAim = !m_continuousTurretAim;
        }));

        Trigger continuousAimTrigger = new Trigger(() -> m_continuousTurretAim);
        
        // Run both Turret Aim and Hood Auto-Adjust simultaneously when toggled ON
        continuousAimTrigger.whileTrue(
            new GooberAlign(rizz, goober).alongWith(new AutoGooba(gooba, rizz))
        );
        
        // Run both Turret Aim and Hood Auto-Adjust simultaneously when Left Bumper is HELD
        operator.leftBumper().and(continuousAimTrigger.negate()).whileTrue(
            new GooberAlign(rizz, goober).alongWith(new AutoGooba(gooba, rizz))
        );

        operator.povUp().whileTrue(new ManualGoobaCommand(gooba, false));
        operator.povDown().whileTrue(new ManualGoobaCommand(gooba, true));
        
        // Manual Turret Commands now use the Constants variable
        operator.povLeft().whileTrue(new ManualTurretCommand(goober, -Constants.Turret.kManualJogSpeed));
        operator.povRight().whileTrue(new ManualTurretCommand(goober, Constants.Turret.kManualJogSpeed));

        // ==========================================
        // --- SYSTEM DEFAULTS ---
        // ==========================================
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Default Shooter Command (Enforces Idle state when not shooting)
        shooter.setDefaultCommand(new RunCommand(() -> {
            if (m_isShooterIdle) {
                shooter.runAtRPM(Constants.Shooter.kIdleRPM);
            } else {
                shooter.stop(); // Truly turn off if idle is disabled
            }
        }, shooter));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public boolean isHubOpenForUs() {
        var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
        String gameData = edu.wpi.first.wpilibj.DriverStation.getGameSpecificMessage();

        if (alliance.isPresent() && gameData != null && !gameData.isEmpty()) {
            char turn = gameData.toUpperCase().charAt(0);
            return (alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) ? (turn == 'R') : (turn == 'B');
        }
        return false;
    }
}