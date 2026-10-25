package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GoobaSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * Continuous hood auto-aim: turns the Limelight's up/down angle to the tag (ty) into a
 * hood position through the calibrated shot map. Runs together with GooberAlign when
 * Operator RB turns auto-aim on.
 *
 * <p>ty is run through a moving-average filter so the hood doesn't chase pixel noise,
 * and the hood simply holds its last position whenever the target drops out of view.
 *
 * <p>Requires only the gooba — do NOT add the Limelight to the requirements or
 * this will fight GooberAlign for it when the two run in parallel.
 */
public class AutoGooba extends Command {
    private final GoobaSubsystem m_gooba;
    private final LimelightSubsystem m_vision;

    // ~5 loops (100 ms) of smoothing on ty
    private final LinearFilter m_tyFilter = LinearFilter.movingAverage(5);
    private boolean m_hadTarget = false;

    public AutoGooba(GoobaSubsystem gooba, LimelightSubsystem vision) {
        this.m_gooba = gooba;
        this.m_vision = vision;
        addRequirements(m_gooba);
    }

    @Override
    public void initialize() {
        // Drop any samples left over from the previous run
        m_tyFilter.reset();
        m_hadTarget = false;
    }

    @Override
    public void execute() {
        // Only move on a valid, ID-filtered target; otherwise hold position.
        if (!m_vision.isValidTarget()) {
            m_hadTarget = false;
            return;
        }

        // Found the tag again: forget the samples from the last time we saw it, and
        // start the filter at the real value so the hood doesn't swing toward ty = 0
        double ty = m_vision.getTY();
        if (!m_hadTarget) {
            m_tyFilter.reset();
            for (int i = 0; i < 4; i++) {
                m_tyFilter.calculate(ty);
            }
        }
        m_hadTarget = true;

        double smoothedTy = m_tyFilter.calculate(ty);
        double targetRotations = m_gooba.getRotationForTy(smoothedTy);

        m_gooba.setPosition(targetRotations);

        // Calibration aids: compare these while recording shot map points
        SmartDashboard.putNumber("AutoGooba ty", smoothedTy);
        SmartDashboard.putNumber("AutoGooba Target Rot", targetRotations);
    }

    @Override
    public boolean isFinished() {
        return false; // run until interrupted (button release or auto timeout)
    }
}
