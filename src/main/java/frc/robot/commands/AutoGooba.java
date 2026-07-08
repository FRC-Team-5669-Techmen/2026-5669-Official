package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GoobaSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * Continuous hood auto-aim: converts the Limelight distance-to-target into a
 * hood position through the calibrated shot map.
 *
 * <p>The distance is run through a moving-average filter so the hood doesn't
 * chase pixel noise, and the hood simply holds its last position whenever the
 * target drops out of view.
 *
 * <p>Requires only the gooba — do NOT add the Limelight to the requirements or
 * this will fight GooberAlign for it when the two run in parallel.
 */
public class AutoGooba extends Command {
    private final GoobaSubsystem m_gooba;
    private final LimelightSubsystem m_vision;

    // ~5 loops (100 ms) of smoothing on the measured distance
    private final LinearFilter m_distanceFilter = LinearFilter.movingAverage(5);

    public AutoGooba(GoobaSubsystem gooba, LimelightSubsystem vision) {
        this.m_gooba = gooba;
        this.m_vision = vision;
        addRequirements(m_gooba);
    }

    @Override
    public void initialize() {
        // Drop any samples left over from the previous run
        m_distanceFilter.reset();
    }

    @Override
    public void execute() {
        // Only move on a valid, ID-filtered target; otherwise hold position.
        if (!m_vision.isValidTarget()) {
            return;
        }

        // Distance can still read 0.0 if the target drops between the check above
        // and this NT read, or if the mount-angle geometry has no solution — never
        // let those samples into the filter or the hood dives to the map minimum.
        double measured = m_vision.distanceToTarget();
        if (measured <= 0.0) {
            return;
        }

        double distance = m_distanceFilter.calculate(measured);
        double targetRotations = m_gooba.getRotationForDistance(distance);

        m_gooba.setPosition(targetRotations);

        // Calibration aids: compare these while recording shot map points
        SmartDashboard.putNumber("AutoGooba Distance (m)", distance);
        SmartDashboard.putNumber("AutoGooba Target Rot", targetRotations);
    }

    @Override
    public boolean isFinished() {
        return false; // run until interrupted (button release or auto timeout)
    }
}
