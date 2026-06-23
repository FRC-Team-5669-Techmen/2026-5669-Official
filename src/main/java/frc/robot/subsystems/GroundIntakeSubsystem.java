package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GroundIntakeSubsystem extends SubsystemBase {
    private final TalonFX m_motor;
    private final DutyCycleOut m_request = new DutyCycleOut(0);

    public GroundIntakeSubsystem() {
        m_motor = new TalonFX(Constants.GroundIntake.kMotorId);
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // REDLINE LIMITS
        CurrentLimitsConfigs currentLimits = config.CurrentLimits;
        // Supply capped just under the PDH channel breaker rating so the
        // software limits gracefully instead of tripping the breaker under load.
        currentLimits.SupplyCurrentLimit = Constants.GroundIntake.kSupplyCurrentLimit;
        currentLimits.SupplyCurrentLimitEnable = true;

        // Higher stator gives torque headroom against compression; supply still caps total draw.
        currentLimits.StatorCurrentLimit = 120.0;
        currentLimits.StatorCurrentLimitEnable = true;

        config.MotorOutput.Inverted = Constants.GroundIntake.kInverted ? 
            InvertedValue.Clockwise_Positive : 
            InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast; 

        m_motor.getConfigurator().apply(config);
    }

    public void runIntake(double speed) {
        m_motor.setControl(m_request.withOutput(speed));
    }

    public void stop() {
        m_motor.stopMotor();
    }
}