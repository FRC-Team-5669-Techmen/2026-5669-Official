package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShooterIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

//Authentically Coded By Lukas Deusch - Senior 

public class FuelHandlingCommand extends Command {
    private final IndexSubsystem index;
    private final ShooterIntakeSubsystem shooterIntake;
    private final ShooterSubsystem shooter;
    private final boolean isForward;

    /**
     * Creates a new FuelHandlingCommand.
     * @param index The Index subsystem
     * @param shooterIntake The Shooter Intake subsystem
     * @param shooter The Shooter subsystem
     * @param isForward True to run intake/shoot sequence (Forward), False to run rewind sequence (Reverse)
     */
    public FuelHandlingCommand(IndexSubsystem index, ShooterIntakeSubsystem shooterIntake, ShooterSubsystem shooter, boolean isForward) {
        this.index = index;
        this.shooterIntake = shooterIntake;
        this.shooter = shooter;
        this.isForward = isForward;

        // Declare subsystem dependencies
        addRequirements(index, shooterIntake, shooter);
    }

    @Override
    public void execute() {
        double indexSpeed;
        double intakeSpeed;
        double shooterSpeed;

        if (isForward) {
            // "Intake" / Shooting direction (Positive constants)
            indexSpeed = Constants.Index.kForwardSpeed;
            intakeSpeed = Constants.ShooterIntake.kForwardSpeed;
            shooterSpeed = Constants.Shooter.kForwardSpeed;
        } else {
            // "Rewind" / Clearing jam direction (Negative constants)
            indexSpeed = Constants.Index.kReverseSpeed;
            intakeSpeed = Constants.ShooterIntake.kReverseSpeed;
            shooterSpeed = Constants.Shooter.kReverseSpeed;
        }

        // Run all motors in unison
        index.run(indexSpeed);
        shooterIntake.run(intakeSpeed);
        shooter.runOpenLoop(shooterSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop all motors when the command ends
        index.stop();
        shooterIntake.stop();
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted (button release)
    }
}