package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;

/**
 * Climber: an elevator that rises, latches on, then pulls the robot up (Talon FX 19).
 * Hold Driver D-pad Up/Down to move it, release to stop. Positive = UP.
 *
 * <p>No soft limits yet. Read the top and bottom numbers off the dashboard
 * ("Elevator Position (rot)") so we can add them next time.
 */
public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(Elevator.kMotorId);

    // FOC off: this motor has no Phoenix Pro license, so it would refuse FOC commands.
    private final VoltageOut request = new VoltageOut(0).withEnableFOC(false);

    public ElevatorSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake; // holds still when released
        config.MotorOutput.Inverted = Elevator.kElevatorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.StatorCurrentLimit = Elevator.kStatorCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        motor.getConfigurator().apply(config);

        // The elevator always starts fully down: call this spot 0.
        motor.setPosition(0);
    }

    /** Positive volts = up, negative = down. */
    public void setVolts(double volts) {
        motor.setControl(request.withOutput(volts));
    }

    public void stop() {
        motor.stopMotor();
    }

    /** Driver D-pad Up: hold to climb up, release to stop. */
    public Command upCommand() {
        return startEnd(() -> setVolts(Elevator.kElevatorVolts), this::stop);
    }

    /** Driver D-pad Down: hold to climb down, release to stop. */
    public Command downCommand() {
        return startEnd(() -> setVolts(-Elevator.kElevatorVolts), this::stop);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Position (rot)", motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Amps", motor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Speed (rot/s)", motor.getVelocity().getValueAsDouble());
    }
}
