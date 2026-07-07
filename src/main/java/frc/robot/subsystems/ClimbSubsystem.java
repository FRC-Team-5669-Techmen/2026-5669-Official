package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Climb winch: Kraken X60 through a 1:100 gearbox with a ratchet mechanism. */
public class ClimbSubsystem extends SubsystemBase {
    private final TalonFX m_motor;
    private final VoltageOut m_request = new VoltageOut(0);

    public ClimbSubsystem() {
        m_motor = new TalonFX(Constants.Climb.kMotorId);
        m_motor.setPosition(0);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimit = Constants.Climb.kSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 150.0;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 1.0;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        config.MotorOutput.Inverted = Constants.Climb.kMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        m_motor.getConfigurator().apply(config);
    }

    /** @param speed percent output (-1 to 1), applied as voltage */
    public void runMotor(double speed) {
        m_motor.setControl(m_request.withOutput(speed * 12.0));
    }

    public void stop() {
        m_motor.stopMotor();
    }
}
