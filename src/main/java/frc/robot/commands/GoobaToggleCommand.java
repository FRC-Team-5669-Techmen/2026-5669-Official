package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GoobaSubsystem;
import frc.robot.Constants;

public class GoobaToggleCommand extends Command {
    private final GoobaSubsystem m_gooba;
    private final boolean m_deploy; 

    public GoobaToggleCommand(GoobaSubsystem gooba, boolean deploy) {
        m_gooba = gooba;
        m_deploy = deploy;
        addRequirements(gooba);
    }

    @Override
    public void initialize() {
        if (m_deploy) {
            m_gooba.setPosition(Constants.Gooba.kPositionDeployed);
        } else {
            m_gooba.setPosition(Constants.Gooba.kPositionStowed);
        }
    }

    @Override
    public boolean isFinished() {
        return true; 
    }
}