package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(61); // Replace 10 with your CAN ID
    private final DutyCycleOut request = new DutyCycleOut(0);

    public TurretSubsystem() {
        // Optional: Set current limits so you don't snap anything if it hits a hard stop
        // motor.getConfigurator().apply(new TalonFXConfiguration().withCurrentLimits(...));
    }

    public void setMotorSpeed(double percent) {
        // Kraken X60s are fast; you might want to multiply percent by 0.5 
        // here to scale the max speed down for testing.
        motor.setControl(request.withOutput(percent * 0.25));
    }

    public void stop() {
        motor.stopMotor();
    }
}
