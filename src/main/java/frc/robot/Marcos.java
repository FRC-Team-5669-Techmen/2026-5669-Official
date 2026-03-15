package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class Marcos {
    /** Registers all commands for PathPlanner to see. */
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
        );

        // Spins up shooter at slower speed (for closer shots)
        NamedCommands.registerCommand("spinUpShooterSlow",
            new RunShooterCommand(shooter, Constants.Shooter.kslowTargetRPM)
        );

        // Runs the full shooting sequence: index + shooterIntake + shooter forward.
        // Give it a timeout in the PathPlanner GUI (1.5-2.5 seconds is typical).
        NamedCommands.registerCommand("shoot",
            new FuelHandlingCommand(index, shooterIntake, shooter, true)
        );

        // Stops the shooter wheels
        NamedCommands.registerCommand("stopShooter",
            new InstantCommand(() -> shooter.stop(), shooter)
        );

        // --- INTAKE (Fuel Handling Reverse = picking up fuel) ---
        // Runs index + shooterIntake + shooter in reverse to pull fuel in.
        // Give it a timeout in PathPlanner GUI (2-3 seconds).
        NamedCommands.registerCommand("intake",
            new FuelHandlingCommand(index, shooterIntake, shooter, false)
        );

        // --- GROUND INTAKE ---
        // Runs the ground intake roller. Give it a timeout in PathPlanner.
        NamedCommands.registerCommand("runGroundIntake",
            new RunGroundIntakeCommand(groundIntake)
        );

        // --- GOOBA (Hood/Arc) ---
        // Deploys the gooba to shooting position
        NamedCommands.registerCommand("deployGooba",
            new GoobaToggleCommand(gooba, true)
        );

        // Stows the gooba back to resting position
        NamedCommands.registerCommand("stowGooba",
            new GoobaToggleCommand(gooba, false)
        );

        // Auto-aims the gooba using Limelight distance.
        // Use as a deadline command or give it a timeout in PathPlanner.
        NamedCommands.registerCommand("autoAimGooba",
            new AutoGooba(gooba, rizz)
        );

        // --- TURRET ---
        // Auto-aims the turret using MariosEar (Limelight + USB cameras).
        // Give it a timeout in PathPlanner (1.5-2.5 seconds).
        NamedCommands.registerCommand("aimTurret",
            new Mariosearcommand(brick, goober)
        );

        // Aligns turret using only the Limelight (no USB cameras).
        NamedCommands.registerCommand("alignTurret",
            new GooberAlign(rizz, goober)
        );

        // --- PNEUMATICS ---
        // Toggle piston 1
        NamedCommands.registerCommand("togglePiston1",
            new TogglePneumaticCommand(piston1)
        );

        // Toggle piston 2
        NamedCommands.registerCommand("togglePiston2",
            new TogglePneumaticCommand(piston2)
        );

        // --- STOP COMMANDS (used by autonomous routines) ---
        // Stops the ground intake roller
        NamedCommands.registerCommand("stopGroundIntake",
            new InstantCommand(() -> groundIntake.stop(), groundIntake)
        );

        // Stops the index and shooter intake (feeder path)
        NamedCommands.registerCommand("stopIntake",
            new InstantCommand(() -> {
                index.stop();
                shooterIntake.stop();
            }, index, shooterIntake)
        );
    }
}