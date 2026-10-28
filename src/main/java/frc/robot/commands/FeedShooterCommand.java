package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShooterIntakeSubsystem;

public class FeedShooterCommand extends Command {
    private final IndexSubsystem index;
    private final ShooterIntakeSubsystem shooterIntake;

    private final java.util.function.BooleanSupplier ready;
    private boolean started = false;

    public FeedShooterCommand(IndexSubsystem index, ShooterIntakeSubsystem shooterIntake) {
        this(index, shooterIntake, () -> true);
    }

    /**
     * @param ready feeding starts once this is true (the wheel is at speed), then keeps going
     *              through the speed dip each ball causes
     */
    public FeedShooterCommand(IndexSubsystem index, ShooterIntakeSubsystem shooterIntake,
                              java.util.function.BooleanSupplier ready) {
        this.ready = ready;
        this.index = index;
        this.shooterIntake = shooterIntake;
        
        // Only requires the feed mechanisms, leaving the shooter free to run independently
        addRequirements(index, shooterIntake);
    }

    @Override
    public void initialize() { started = false; }

    @Override
    public void execute() {
        if (!started && !ready.getAsBoolean()) { index.stop(); shooterIntake.stop(); return; }
        started = true;
        index.feed(); // forward with automatic unjam
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