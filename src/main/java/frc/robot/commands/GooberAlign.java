package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Goober;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * PID-aligns the turret onto the Limelight target. While no valid target is in
 * view, the turret sweeps back and forth between its soft limits hunting for one.
 *
 * <p>Never finishes on its own — bind with whileTrue() in teleop or add
 * .withTimeout() in autos (Marcos already does).
 */
public class GooberAlign extends Command {
    private final LimelightSubsystem limelight;
    private final Goober turret;

    private int seekDirection = 1; // 1 = sweep right, -1 = sweep left

    public GooberAlign(LimelightSubsystem limelight, Goober turret) {
        this.limelight = limelight;
        this.turret = turret;
        addRequirements(turret, limelight);
    }

    @Override
    public void initialize() {
        seekDirection = 1;
        // Drop stale samples from the previous alignment run
        turret.resetAimFilter();
    }

    @Override
    public void execute() {
        if (limelight.isValidTarget()) {
            // Track the target
            turret.aimAtTarget(-limelight.getCorrectedTX());
        } else {
            // Hunt for the target: bounce between the firmware soft limits (+10 / -25)
            double currentPosition = turret.getPosition();
            if (currentPosition >= 9.5) {
                seekDirection = -1; // Hit right limit, sweep left
            } else if (currentPosition <= -24.5) {
                seekDirection = 1;  // Hit left limit, sweep right
            }
            turret.setMotorSpeed(seekDirection * Constants.Turret.kSearchSpeed);
        }
    }

    @Override
    public boolean isFinished() {
        // Must be false: finishing when the target drops out would kill the
        // search sweep before it ever finds anything.
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }
}
