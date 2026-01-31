package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class RunShooterCommand extends Command {
    private final ShooterSubsystem m_shooter;
    private final double m_targetRPM;

    public RunShooterCommand(ShooterSubsystem shooter, double targetRPM) {
        m_shooter = shooter;
        m_targetRPM = targetRPM;
        addRequirements(m_shooter);
    }
    @Override
    public void execute() {
        
        m_shooter.runAtRPM(m_targetRPM);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
    }
}