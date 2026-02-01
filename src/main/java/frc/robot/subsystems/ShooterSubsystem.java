package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    //ID init
    private final TalonFX leader = new TalonFX(Constants.Shooter.kLeaderId);
    private final TalonFX follower = new TalonFX(Constants.Shooter.kFollowerId);
    
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);

    public ShooterSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        //ConstG
        config.Slot0.kP = Constants.Shooter.kP; 
        config.Slot0.kV = Constants.Shooter.kV;
        config.Slot0.kI = Constants.Shooter.kI;
        config.Slot0.kD = Constants.Shooter.kD;

        //Ramps
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Shooter.kVoltageRampPeriod;
        config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Shooter.kDutyCycleRampPeriod;
        
        leader.getConfigurator().apply(config);

        follower.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    public void runAtRPM(double rpm) {
        double rps = rpm / 60.0;
        leader.setControl(m_velocityRequest.withVelocity(rps));
    }

    public void stop() {
        leader.stopMotor();
    }
}