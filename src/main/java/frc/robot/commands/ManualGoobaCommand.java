package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.GoobaSubsystem;

/**
 * Jogs the hood (gooba) position up or down in small steps while held.
 * Motion Magic holds the final position after release.
 */
public class ManualGoobaCommand extends Command {
    private final GoobaSubsystem m_gooba;
    private final boolean m_increase;
    private double m_targetPosition;

    // Jog rate: 0.02 rot/tick * 50 ticks/sec = 1.0 rotation per second.
    private static final double kJogStep = 0.02;

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
        // Motion Magic automatically holds the last commanded target position.
    }
}
