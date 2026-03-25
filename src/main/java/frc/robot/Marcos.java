package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class Marcos {
    public static void registerNamedCommands(
        ShooterSubsystem shooter, 
        IndexSubsystem index, 
        ShooterIntakeSubsystem shooterIntake,
        GoobaSubsystem gooba,
        Goober goober,
        LimelightSubsystem rizz,
        MariosEar brick,
        GroundIntakeSubsystem groundIntake,
        PneumaticSubsystem piston1,
        PneumaticSubsystem piston2
    ) {
         NamedCommands.registerCommand("spinUpShooter",
            new RunShooterCommand(shooter, Constants.Shooter.kfastTargetRPM)
        .withTimeout(1.5)
        );

        NamedCommands.registerCommand("spinUpShooterSlow",
            new RunShooterCommand(shooter, Constants.Shooter.kslowTargetRPM)
        .withTimeout(1.5)
        );

        NamedCommands.registerCommand("score",
            new SequentialCommandGroup(
                new GooberAlign(rizz, goober).withTimeout(2.0),
                new RunShooterCommand(shooter, Constants.Shooter.kfastTargetRPM).withTimeout(1.5),
                new FuelHandlingCommand(index, shooterIntake, shooter, true).withTimeout(2.0),
                new InstantCommand(() -> shooter.stop(), shooter)
            )
        );

        // ADDED TIMEOUT TO PREVENT AUTO HANGS
        NamedCommands.registerCommand("shoot",
            new FuelHandlingCommand(index, shooterIntake, shooter, true).withTimeout(2.0)
        );

        NamedCommands.registerCommand("stopShooter",
            new InstantCommand(() -> shooter.stop(), shooter)
        );

        // ADDED TIMEOUT
        NamedCommands.registerCommand("intake",
            new FuelHandlingCommand(index, shooterIntake, shooter, false).withTimeout(2.0)
        );

        // Runs ground intake + feed motors together so the game piece gets loaded
        NamedCommands.registerCommand("runGroundIntake",
            new RunGroundIntakeCommand(groundIntake)
                .alongWith(new FeedShooterCommand(index, shooterIntake))
                .withTimeout(5.0)
        );

        NamedCommands.registerCommand("deployGooba",
            new GoobaToggleCommand(gooba, true)
        );

        NamedCommands.registerCommand("stowGooba",
            new GoobaToggleCommand(gooba, false)
        );

        NamedCommands.registerCommand("autoAimGooba",
            new AutoGooba(gooba, rizz).withTimeout(2.0)
        );

        // ADDED TIMEOUT
        NamedCommands.registerCommand("aimTurret",
            new Mariosearcommand(brick, goober).withTimeout(2.0)
        );

        // ADDED TIMEOUT
        NamedCommands.registerCommand("alignTurret",
            new GooberAlign(rizz, goober).withTimeout(2.0)
        );

        NamedCommands.registerCommand("togglePiston1",
            new TogglePneumaticCommand(piston1)
        );

        NamedCommands.registerCommand("togglePiston2",
            new TogglePneumaticCommand(piston2)
        );
    }
}