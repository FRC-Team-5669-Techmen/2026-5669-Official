package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(Constants.Turret.kMotorId);
    private final DutyCycleOut request = new DutyCycleOut(0);

    public TurretSubsystem() {
    }

    public void setMotorSpeed(double percent) {
        motor.setControl(request.withOutput(percent * Constants.Turret.kSpeedMultiplier));
    }

    public void stop() {
        motor.stopMotor();
    }
}