package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShooterIntakeSubsystem;

public class FeedShooterCommand extends Command {
    private final IndexSubsystem index;
    private final ShooterIntakeSubsystem shooterIntake;

    public FeedShooterCommand(IndexSubsystem index, ShooterIntakeSubsystem shooterIntake) {
        this.index = index;
        this.shooterIntake = shooterIntake;
        
        // Only requires the feed mechanisms, leaving the shooter free to run independently
        addRequirements(index, shooterIntake);
    }

    @Override
    public void execute() {
        index.run(frc.robot.subsystems.IndexSubsystem.forwardVolts());
        shooterIntake.feed();
    }

    @Override
    public void end(boolean interrupted) {
        index.stop();
        shooterIntake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}