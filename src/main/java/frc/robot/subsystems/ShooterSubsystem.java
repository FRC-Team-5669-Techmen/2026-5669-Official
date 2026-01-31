package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX leader = new TalonFX(16);
    private final TalonFX follower = new TalonFX(17);
    
    // Velocity control object
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);

    public ShooterSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();

     
        config.Slot0.kP = -0.11; 
        config.Slot0.kV = 0;

        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 100.0;
        config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 100.0;
        
        leader.getConfigurator().apply(config);

      
        follower.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    public void runAtRPM(double rpm) {
        // Convert RPM to Rotations per Second
        double rps = rpm / 60.0;
        leader.setControl(m_velocityRequest.withVelocity(rps));
    }

    public void stop() {
        leader.stopMotor();
    }
}