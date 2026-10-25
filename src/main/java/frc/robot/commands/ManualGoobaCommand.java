package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.GoobaSubsystem;

/**
 * Operator D-pad Up/Down: jogs the hood (gooba) position up or down in small steps while held.
 * Motion Magic holds the final position after release.
 */
public class ManualGoobaCommand extends Command {
    private final GoobaSubsystem m_gooba;
    private final boolean m_increase;
    private double m_targetPosition;

    // Jog rate: kManualStep rot/tick * 50 ticks/sec (0.04 = 2 rotations per second).
    private static final double kJogStep = Constants.Gooba.kManualStep;
    // How far (rotations) the jog target may run ahead of the real position
    private static final double kJogLead = 0.5;

    /**
     * @param gooba    The hood subsystem
     * @param increase true to jog the position up (toward deployed),
     *                 false to jog it down (toward stowed)
     */
    public ManualGoobaCommand(GoobaSubsystem gooba, boolean increase) {
        m_gooba = gooba;
        m_increase = increase;
        addRequirements(gooba);
    }

    @Override
    public void initialize() {
        // Start jogging from wherever the hood actually is right now.
        m_targetPosition = m_gooba.getPosition();
    }

    @Override
    public void execute() {
        if (m_increase) {
            m_targetPosition += kJogStep;
        } else {
            m_targetPosition -= kJogStep;
        }

        // Never let the target run more than kJogLead ahead of where the hood really is, so
        // holding the D-pad against an end (or a jam) doesn't pile up a far-away target
        double position = m_gooba.getPosition();
        m_targetPosition = Math.max(position - kJogLead, Math.min(position + kJogLead, m_targetPosition));
        // Stay inside the travel, unless the limits are switched off to measure
        if (!m_gooba.limitsOff()) {
            m_targetPosition = Math.max(Constants.Gooba.kBottomPosition,
                Math.min(Constants.Gooba.kTopPosition, m_targetPosition));
        }
        m_gooba.setPosition(m_targetPosition);

        // Write this number down when you make a perfect shot!
        SmartDashboard.putNumber("Gooba Fine-Tune Position", m_targetPosition);
    }

    @Override
    public boolean isFinished() {
        return false; // Run continuously while the button is held
    }

    @Override
    public void end(boolean interrupted) {
        // Hold where the hood actually is now (not a target it may never reach)
        m_gooba.setPosition(m_gooba.getPosition());
    }
}
