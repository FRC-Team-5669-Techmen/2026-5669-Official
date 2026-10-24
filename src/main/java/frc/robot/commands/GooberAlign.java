package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Goober;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * PID-aligns the turret onto the Limelight target. While no valid target is in
 * view, the turret sweeps back and forth across its range hunting for one.
 * Does nothing until the turret's far end (kEndPosition) is measured.
 *
 * <p>Never finishes on its own — bind with whileTrue() in teleop or add
 * .withTimeout() in autos (Marcos already does).
 */
public class GooberAlign extends Command {
    private final LimelightSubsystem limelight;
    private final Goober turret;

    private int seekDirection = 1; // 1 = sweep right (away from HOME), -1 = sweep left
    private boolean hadTarget = false;

    public GooberAlign(LimelightSubsystem limelight, Goober turret) {
        this.limelight = limelight;
        this.turret = turret;
        addRequirements(turret, limelight);
    }

    @Override
    public void initialize() {
        // Sweep toward whichever end is further away first
        double middle = (turret.aimMinPosition() + turret.aimMaxPosition()) / 2.0;
        seekDirection = turret.getPosition() < middle ? 1 : -1;
        hadTarget = false;
        // Drop stale samples from the previous alignment run
        turret.resetAimFilter();
    }

    @Override
    public void execute() {
        // Until the far end is measured the code doesn't know where the wires are,
        // so auto-aim must not move the turret at all.
        if (!turret.endMeasured()) {
            turret.stop();
            turret.setAimStatus("OFF: far end not set");
            return;
        }

        if (limelight.isValidTarget()) {
            // Found the tag again: forget the samples from the last time we saw it
            if (!hadTarget) {
                turret.resetAimFilter();
            }
            hadTarget = true;
            // The camera is centered on the shooter and turns with it, so raw tx aims the
            // turret directly. tx > 0 = tag on the RIGHT, and positive output turns RIGHT.
            turret.aimAtTarget(limelight.getTX());
        } else {
            hadTarget = false;
            // Hunt for the target: bounce between the aim limits (just inside both ends)
            double currentPosition = turret.getPosition();
            if (currentPosition >= turret.aimMaxPosition()) {
                seekDirection = -1; // Reached the far end, sweep back left
            } else if (currentPosition <= turret.aimMinPosition()) {
                seekDirection = 1;  // Reached HOME, sweep right
            }
            turret.setMotorSpeed(seekDirection * Constants.Turret.kSearchSpeed);
            turret.setAimStatus("SEARCHING");
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
        turret.setAimStatus("OFF");
    }
}
