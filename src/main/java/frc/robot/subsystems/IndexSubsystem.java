package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(Constants.Index.kMotorId);
    
    // FOC off: this motor has no Phoenix Pro license, and an unlicensed motor refuses
    // FOC commands. (Only the shooter motors 16 and 17 are licensed.)
    private final VoltageOut request = new VoltageOut(0).withEnableFOC(false);

    public IndexSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = Constants.Index.kInverted ? 
            com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive : 
            com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // SAFETY GLOVES OFF: These limits match your main shooter flywheel
        CurrentLimitsConfigs currentLimits = config.CurrentLimits;
        
        // Let it pull up to 80 Amps from the battery instantly
        currentLimits.SupplyCurrentLimit = 80.0; 
        currentLimits.SupplyCurrentLimitEnable = true;

        // Let it push up to 160 Amps of torque to the physical coils
        currentLimits.StatorCurrentLimit = 160.0; 
        currentLimits.StatorCurrentLimitEnable = true;
        
        motor.getConfigurator().apply(config);
    }

    public void run(double volts) {
        motor.setControl(request.withOutput(volts));
    }

    public void stop() {
        motor.stopMotor();
    }

    /** "Spindex Speed (rot/s)" and "Spindex Amps": a slow speed with high amps = jammed or geared too low. */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Spindex Speed (rot/s)", motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Spindex Amps", motor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Spindex Volts", motor.getMotorVoltage().getValueAsDouble());
    }
}
