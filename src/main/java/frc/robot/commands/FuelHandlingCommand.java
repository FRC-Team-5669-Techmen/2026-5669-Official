package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShooterIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

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
        double shooterRPM;

        if (isForward) {
            // "Intake" / Shooting direction
            indexSpeed = Constants.Index.kForwardSpeed;
            intakeSpeed = Constants.ShooterIntake.kForwardSpeed;
            shooterRPM = Constants.Shooter.kfastTargetRPM; // 5000 RPM
        } else {
            // "Rewind" / Clearing jam direction
            indexSpeed = Constants.Index.kReverseSpeed;
            intakeSpeed = Constants.ShooterIntake.kReverseSpeed;
            shooterRPM = Constants.Shooter.kReverseRPM; // -1000 RPM
        }

        // Run all motors in unison
        index.run(indexSpeed);
        shooterIntake.run(intakeSpeed);
        
        // Use RPM control (VelocityVoltage) for the shooter
        shooter.runAtRPM(shooterRPM);
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
        return false;
    }
}