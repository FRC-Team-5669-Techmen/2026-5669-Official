package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
                new RunShooterCommand(shooter, Constants.Shooter.kfastTargetRPM).withTimeout(5),
                new FuelHandlingCommand(index, shooterIntake, shooter, true).withTimeout(2.0),
                new InstantCommand(() -> shooter.stop(), shooter)
            )
        );
        
        NamedCommands.registerCommand("lowerIntake",
            new InstantCommand(() -> piston1.extend(), piston1)
        );

        // Score preloaded balls without any vision/limelight tracking
        NamedCommands.registerCommand("scorePreload",
            new SequentialCommandGroup(
                new RunShooterCommand(shooter, Constants.Shooter.kfastTargetRPM).withTimeout(5.5),
                new FuelHandlingCommand(index, shooterIntake, shooter, true).withTimeout(2.5),
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

        // No timeout — runs continuously for the rest of auto when triggered
        NamedCommands.registerCommand("runGroundIntake",
            new RunGroundIntakeCommand(groundIntake)
        );

        // Event markers in .path files use EventTrigger, NOT NamedCommands.
        // This binds the "runGroundIntake" event marker to actually start the command.
        new EventTrigger("runGroundIntake").onTrue(new RunGroundIntakeCommand(groundIntake));

        // Limelight-tracked score: align turret + auto-aim hood, then shoot.
        // Robot must be stopped before calling this — no shooting while moving.
        NamedCommands.registerCommand("visionScore",
            new SequentialCommandGroup(
                // Phase 1: Align turret (GooberAlign) + aim hood (AutoGooba) simultaneously
                new ParallelCommandGroup(
                    new GooberAlign(rizz, goober),
                    new AutoGooba(gooba, rizz)
                ).withTimeout(2.5),
                // Phase 2: Spin up shooter and fire, please work
                new RunShooterCommand(shooter, Constants.Shooter.kfastTargetRPM).withTimeout(5.5),
                new FuelHandlingCommand(index, shooterIntake, shooter, true).withTimeout(2.5),
                new InstantCommand(() -> shooter.stop(), shooter)
            )
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