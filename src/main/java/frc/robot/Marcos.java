package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
        Goober goober,
        LimelightSubsystem limelight,
        MariosEar mariosEar
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

        // Limelight-tracked score: align the turret, then shoot.
        // (Used to aim the hood at the same time; the hood is gone.)
        // Robot must be stopped before calling this — no shooting while moving.
        NamedCommands.registerCommand("visionScore",
            new SequentialCommandGroup(
                // Phase 1: Align turret
                new GooberAlign(limelight, goober).withTimeout(2.5),
                // Phase 2: Spin up shooter and fire
                new RunShooterCommand(shooter, Constants.Shooter.kFastTargetRPM).withTimeout(5.5),
                new FuelHandlingCommand(index, shooterIntake, shooter, true).withTimeout(2.5),
                new InstantCommand(shooter::stop, shooter)
            )
        );

        NamedCommands.registerCommand("aimTurret",
            new MariosEarCommand(mariosEar, goober).withTimeout(2.0)
        );

        NamedCommands.registerCommand("alignTurret",
            new GooberAlign(limelight, goober).withTimeout(2.0)
        );

        // ==========================================
        // PNEUMATICS REMOVED (2026 offseason).
        // These names are still referenced by the deployed 3-piece autos, so they
        // stay registered as no-ops until the autos are rebuilt in PathPlanner.
        // ==========================================
        NamedCommands.registerCommand("lowerIntake", Commands.none());
        NamedCommands.registerCommand("togglePiston1", Commands.none());
        NamedCommands.registerCommand("togglePiston2", Commands.none());

        // ==========================================
        // GROUND INTAKE RETIRED (2026 offseason).
        // The "runGroundIntake" event markers in 5 .path files are trigger-only, so
        // with no EventTrigger bound they simply do nothing. The named command stays
        // registered as a no-op so nothing logs a missing-command warning.
        // (The roller motor is gone; CAN 20 is now the intake pivot leader.)
        // ==========================================
        NamedCommands.registerCommand("runGroundIntake", Commands.none());

        // ==========================================
        // HOOD REMOVED (Oct 2026): no hood motor on the robot. Kept as no-ops so any
        // auto that references them still loads.
        // ==========================================
        NamedCommands.registerCommand("deployGooba", Commands.none());
        NamedCommands.registerCommand("stowGooba", Commands.none());
        NamedCommands.registerCommand("autoAimGooba", Commands.none());
    }
}
