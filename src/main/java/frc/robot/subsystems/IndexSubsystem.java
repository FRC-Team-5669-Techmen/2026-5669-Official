package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//Authentically Coded By Lukas Deusch - Senior 

public class IndexSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(Constants.Index.kMotorId);
    private final DutyCycleOut request = new DutyCycleOut(0);

    public IndexSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = Constants.Index.kInverted ? 
            com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive : 
            com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        motor.getConfigurator().apply(config);
    }

    public void run(double speed) {
        motor.setControl(request.withOutput(speed));
    }

    public void stop() {
        motor.stopMotor();
    }
}