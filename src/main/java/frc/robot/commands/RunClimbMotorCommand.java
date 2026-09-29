package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class RunClimbMotorCommand extends Command {
    private final ClimbSubsystem m_climb;
    private final double m_speed;

    public RunClimbMotorCommand(ClimbSubsystem climb, double speed) {
        m_climb = climb;
        m_speed = speed;
        addRequirements(climb);
    }

    @Override
    public void execute() {
        m_climb.runMotor(m_speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_climb.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs continuously while the button is held down
    }
}