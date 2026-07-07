package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * Registers every PathPlanner NamedCommand and event trigger.
 * Must run BEFORE AutoBuilder.buildAutoChooser() — RobotContainer calls this first.
 *
 * <p>The registered string names are referenced by the .auto/.path files in
 * src/main/deploy/pathplanner — do not rename them without updating the autos.
 */
public class Marcos {
    public static void registerNamedCommands(
        ShooterSubsystem shooter,
        IndexSubsystem index,
        ShooterIntakeSubsystem shooterIntake,
        GoobaSubsystem gooba,
        Goober goober,
        LimelightSubsystem limelight,
        MariosEar mariosEar,
        GroundIntakeSubsystem groundIntake,
        PneumaticSubsystem intakePistons,
        PneumaticSubsystem climbPiston
    ) {
        NamedCommands.registerCommand("spinUpShooter",
            new RunShooterCommand(shooter, Constants.Shooter.kFastTargetRPM)
                .withTimeout(1.5)
        );

        NamedCommands.registerCommand("spinUpShooterSlow",
            new RunShooterCommand(shooter, Constants.Shooter.kSlowTargetRPM)
                .withTimeout(1.5)
        );

        NamedCommands.registerCommand("score",
            new SequentialCommandGroup(
                new GooberAlign(limelight, goober).withTimeout(2.0),
                new RunShooterCommand(shooter, Constants.Shooter.kFastTargetRPM).withTimeout(5),
                new FuelHandlingCommand(index, shooterIntake, shooter, true).withTimeout(2.0),
                new InstantCommand(shooter::stop, shooter)
            )
        );

        NamedCommands.registerCommand("lowerIntake",
            new InstantCommand(intakePistons::extend, intakePistons)
        );

        // Score preloaded balls without any vision/limelight tracking
        NamedCommands.registerCommand("scorePreload",
            new SequentialCommandGroup(
                new RunShooterCommand(shooter, Constants.Shooter.kFastTargetRPM).withTimeout(5.5),
                new FuelHandlingCommand(index, shooterIntake, shooter, true).withTimeout(2.5),
                new InstantCommand(shooter::stop, shooter)
            )
        );

        // Timeout prevents auto hangs
        NamedCommands.registerCommand("shoot",
            new FuelHandlingCommand(index, shooterIntake, shooter, true).withTimeout(2.0)
        );

        NamedCommands.registerCommand("stopShooter",
            new InstantCommand(shooter::stop, shooter)
        );

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
                    new GooberAlign(limelight, goober),
                    new AutoGooba(gooba, limelight)
                ).withTimeout(2.5),
                // Phase 2: Spin up shooter and fire
                new RunShooterCommand(shooter, Constants.Shooter.kFastTargetRPM).withTimeout(5.5),
                new FuelHandlingCommand(index, shooterIntake, shooter, true).withTimeout(2.5),
                new InstantCommand(shooter::stop, shooter)
            )
        );

        NamedCommands.registerCommand("deployGooba",
            new GoobaToggleCommand(gooba, true)
        );

        NamedCommands.registerCommand("stowGooba",
            new GoobaToggleCommand(gooba, false)
        );

        NamedCommands.registerCommand("autoAimGooba",
            new AutoGooba(gooba, limelight).withTimeout(2.0)
        );

        NamedCommands.registerCommand("aimTurret",
            new MariosEarCommand(mariosEar, goober).withTimeout(2.0)
        );

        NamedCommands.registerCommand("alignTurret",
            new GooberAlign(limelight, goober).withTimeout(2.0)
        );

        NamedCommands.registerCommand("togglePiston1",
            new TogglePneumaticCommand(intakePistons)
        );

        NamedCommands.registerCommand("togglePiston2",
            new TogglePneumaticCommand(climbPiston)
        );
    }
}
