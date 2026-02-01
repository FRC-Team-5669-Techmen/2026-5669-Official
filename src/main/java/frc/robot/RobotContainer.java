// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Muhammad Nabeel
// Lukas Deusch 
// Jhonen Hasenbein
// Marcos "Danger" Posada

package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.RunShooterCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;


import frc.robot.generated.TunerConstants;
//Diddy AYO
import frc.robot.Constants;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
    
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
    
    
    private double MaxAngularRate = Constants.Operator.kMaxAngularRate.in(RadiansPerSecond);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            // Use Constants for Deadbands
            .withDeadband(MaxSpeed * Constants.Operator.kDeadband)
            .withRotationalDeadband(MaxAngularRate * Constants.Operator.kRotationalDeadband) 
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 
            
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    
    private final CommandXboxController joystick = new CommandXboxController(Constants.Operator.kDriverControllerPort);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ShooterSubsystem shooter = new ShooterSubsystem();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(Math.pow((-joystick.getLeftY()), 3) * MaxSpeed)
                    .withVelocityY(Math.pow((-joystick.getLeftX()), 3) * MaxSpeed) 
                    .withRotationalRate(Math.pow((-joystick.getRightX()), 3) * MaxAngularRate) 
            )
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        
        joystick.a().whileTrue(new RunShooterCommand(shooter, Constants.Shooter.kfastTargetRPM));
        joystick.rightBumper().whileTrue(new RunShooterCommand(shooter, Constants.Shooter.kslowTargetRPM));
        
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            drivetrain.applyRequest(() ->
                // Use Constants for Auton Speed
                drive.withVelocityX(Constants.Auton.kDriveSpeed)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            
            .withTimeout(Constants.Auton.kTimeoutSeconds),
            drivetrain.applyRequest(() -> idle)
        );
    }
}