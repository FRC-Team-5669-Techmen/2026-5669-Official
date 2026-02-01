package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    // ID init
    private final TalonFX leader = new TalonFX(Constants.Shooter.kLeaderId);
    private final TalonFX follower = new TalonFX(Constants.Shooter.kFollowerId);
    
    // Controls - STRICTLY VELOCITY (RPM)
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);

    public ShooterSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // PID Config
        config.Slot0.kP = Constants.Shooter.kP; 
        config.Slot0.kV = Constants.Shooter.kV;
        config.Slot0.kI = Constants.Shooter.kI;
        config.Slot0.kD = Constants.Shooter.kD;

        // Direction Config
        config.MotorOutput.Inverted = Constants.Shooter.kLeaderInverted ? 
            com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive : 
            com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;

        // Ramps
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Shooter.kVoltageRampPeriod;
        config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Shooter.kDutyCycleRampPeriod;
        
        leader.getConfigurator().apply(config);

        // Follower is strictly aligned to leader
        follower.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    public void runAtRPM(double rpm) {
        // Convert RPM to Rotations Per Second (RPS)
        double rps = rpm / 60.0;
        leader.setControl(m_velocityRequest.withVelocity(rps));
    }

    public void stop() {
        leader.stopMotor();
    }
}