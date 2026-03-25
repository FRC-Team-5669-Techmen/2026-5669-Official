package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX leader = new TalonFX(Constants.Shooter.kLeaderId);
    private final TalonFX follower = new TalonFX(Constants.Shooter.kFollowerId);
    
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
    
    private double currentTargetRpm = 0.0;
    
    // Internal software tracker to guarantee smooth ramps
    private double m_rampedSetpointRpm = 0.0;

    public ShooterSubsystem() {
        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();

        leaderConfig.Slot0.kP = Constants.Shooter.kP; 
        leaderConfig.Slot0.kV = Constants.Shooter.kV;
        leaderConfig.Slot0.kI = Constants.Shooter.kI;
        leaderConfig.Slot0.kD = Constants.Shooter.kD;
        
        followerConfig.Slot0.kP = Constants.Shooter.kP; 
        followerConfig.Slot0.kV = Constants.Shooter.kV;
        followerConfig.Slot0.kI = Constants.Shooter.kI;
        followerConfig.Slot0.kD = Constants.Shooter.kD;

        leaderConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Shooter.kVoltageRampPeriod;
        leaderConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Shooter.kDutyCycleRampPeriod;
        followerConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Shooter.kVoltageRampPeriod;
        followerConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Shooter.kDutyCycleRampPeriod;

        // MAX PERFORMANCE LIMITS: Tied directly to Constants for easy tuning
        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimit = Constants.Shooter.kSupplyCurrentLimit; 
        currentLimits.SupplyCurrentLimitEnable = true;

        currentLimits.StatorCurrentLimit = Constants.Shooter.kStatorCurrentLimit; 
        currentLimits.StatorCurrentLimitEnable = true;

        leaderConfig.CurrentLimits = currentLimits;
        followerConfig.CurrentLimits = currentLimits;

        leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        leaderConfig.MotorOutput.Inverted = Constants.Shooter.kLeaderInverted ? 
            com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive : 
            com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;

        followerConfig.MotorOutput.Inverted = Constants.Shooter.kFollowerInverted ? 
            com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive : 
            com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;

        leader.getConfigurator().apply(leaderConfig);
        follower.getConfigurator().apply(followerConfig);
    }

    public double getCurrentRpm() {
        // Phoenix 6 returns Rotations per Second, so we multiply by 60 to get RPM
        return leader.getVelocity().getValueAsDouble() * 60.0;
    }

    public void runAtRPM(double targetRpm) {
        // SAFETY CLAMP: Force the requested RPM to stay inside our limits (-3500 to 3500)
        targetRpm = MathUtil.clamp(targetRpm, -Constants.Shooter.kMaxRPM, Constants.Shooter.kMaxRPM);
        
        currentTargetRpm = targetRpm;
        
        // --- FIXED SPEED MANAGEMENT LOGIC ---
        if (m_rampedSetpointRpm > targetRpm + 50.0) {
            // 1. SMOOTH SPIN-DOWN
            m_rampedSetpointRpm -= Constants.Shooter.kDecelerateStep; 
            if (m_rampedSetpointRpm < targetRpm) {
                m_rampedSetpointRpm = targetRpm;
            }
            
        } else if (Math.abs(targetRpm - Constants.Shooter.kIdleRPM) < 1.0 && m_rampedSetpointRpm < targetRpm) {
            // 2. SMOOTH IDLE SPIN-UP
            m_rampedSetpointRpm += Constants.Shooter.kIdleAccelerateStep;
            if (m_rampedSetpointRpm > targetRpm) {
                m_rampedSetpointRpm = targetRpm;
            }
            
        } else {
            // 3. ACTUAL SHOOTING (Instant application for max acceleration)
            m_rampedSetpointRpm = targetRpm;
        }

        // Send the managed software setpoint to the TalonFX
        leader.setControl(m_velocityRequest.withVelocity(m_rampedSetpointRpm / 60.0));
        follower.setControl(m_velocityRequest.withVelocity(m_rampedSetpointRpm / 60.0));
    }

    public void testLeaderOnly(double rpm) {
        rpm = MathUtil.clamp(rpm, -Constants.Shooter.kMaxRPM, Constants.Shooter.kMaxRPM);
        currentTargetRpm = rpm;
        m_rampedSetpointRpm = getCurrentRpm(); // THE FIX: Sync to reality
        leader.setControl(m_velocityRequest.withVelocity(rpm / 60.0));
        follower.stopMotor(); 
    }

    public void testFollowerOnly(double rpm) {
        rpm = MathUtil.clamp(rpm, -Constants.Shooter.kMaxRPM, Constants.Shooter.kMaxRPM);
        currentTargetRpm = rpm;
        m_rampedSetpointRpm = getCurrentRpm(); // THE FIX: Sync to reality
        follower.setControl(m_velocityRequest.withVelocity(rpm / 60.0));
        leader.stopMotor(); 
    }

    public void stop() {
        currentTargetRpm = 0.0;
        
        // THE FIX: Instead of dropping our tracker to 0 instantly, we sync it 
        // to the physical wheel speed. This guarantees a butter-smooth handoff 
        // to the Idle command when the trigger is released.
        m_rampedSetpointRpm = getCurrentRpm(); 
        
        leader.stopMotor();
        follower.stopMotor();
    }

    @Override
    public void periodic() {
        
        SmartDashboard.putNumber("Shooter Actual RPM", getCurrentRpm());
        SmartDashboard.putNumber("Shooter Target RPM", currentTargetRpm);
        
        // Grab the live stator current (torque) so you can physically see the acceleration effort
        double currentDraw = Math.round(leader.getStatorCurrent().getValueAsDouble() * 10.0) / 10.0;
        

      //  System.out.println("SHOOTER RPM: " + Math.round(getCurrentRpm()) + 
       //                    " | TARGET: " + currentTargetRpm + 
       //                    " | TORQUE AMPS: " + currentDraw);

                         //  " | TARGET: " + currentTargetRpm + 
                           //" | RAMPER: " + Math.round(m_rampedSetpointRpm) +
                         //  " | TORQUE AMPS: " + currentDraw);
    }
}
