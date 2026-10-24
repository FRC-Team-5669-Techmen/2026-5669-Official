package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Goober;

public class ManualTurretCommand extends Command {
    private final Goober m_turret;
    private final double m_speed;

    /**
     * @param turret The turret subsystem (Goober)
     * @param speed  The percent output to run the motor (-1.0 to 1.0), positive = turn RIGHT.
     *               The turret's soft limits stop it at HOME and at the far end.
     */
    public ManualTurretCommand(Goober turret, double speed) {
        m_turret = turret;
        m_speed = speed;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        // Run the turret motor at the specified manual speed
        m_turret.setMotorSpeed(m_speed);
    }

    @Override
    public boolean isFinished() {
        return false; // Run continuously while the D-Pad is held
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the motor immediately when the driver lets go of the button
        m_turret.stop();
    }
}