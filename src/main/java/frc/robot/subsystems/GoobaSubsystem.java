package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class GoobaSubsystem extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(Constants.Gooba.kMotorId);
    
    // The interpolation map that handles the math between your calibrated points
    private final InterpolatingDoubleTreeMap shotMap = new InterpolatingDoubleTreeMap();
    
    // Motion Magic Request (Smooth position control)
    private final MotionMagicVoltage m_positionControl = new MotionMagicVoltage(0);

    public GoobaSubsystem() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        // PID for Position
        configs.Slot0.kP = Constants.Gooba.kP;
        configs.Slot0.kI = Constants.Gooba.kI;
        configs.Slot0.kD = Constants.Gooba.kD;
        
        // Motion Magic parameters
        configs.MotionMagic.MotionMagicCruiseVelocity = Constants.Gooba.kCruiseVelocity;
        configs.MotionMagic.MotionMagicAcceleration = Constants.Gooba.kAcceleration;

        // Current Limits (Essential for Kraken X44 safety)
        configs.CurrentLimits.SupplyCurrentLimit = Constants.Gooba.kSupplyCurrentLimit;
        configs.CurrentLimits.SupplyCurrentLimitEnable = true;

        // --- FIRMWARE SOFT LIMITS ---
        configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
        configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 10.7; 
        configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // ----------------------------

        m_motor.getConfigurator().apply(configs);
        
        m_motor.setNeutralMode(NeutralModeValue.Brake);
        m_motor.setPosition(0);

        // ==========================================
        // YOUR REAL INTERPOLATION MAP
        // Format: shotMap.put(Limelight_TX, Hood_Position);
        // ==========================================
        
        // --- SAFETY DEFAULT ---
        // This guarantees the map is never empty so the code doesn't crash
        // before you finish your calibration!
        shotMap.put(-15.6, 0.0); 
        shotMap.put(-24.6, 3.5); 


        // TODO: Add your real data points here once you calibrate them!
        // shotMap.put(YOUR_TX_1, YOUR_HOOD_POS_1);
        // shotMap.put(YOUR_TX_2, YOUR_HOOD_POS_2);
        
    }

    public void setPosition(double rotations) {
        // Safe movement inside the 0.0 to 10.7 soft limits
        m_motor.setControl(m_positionControl.withPosition(rotations));
    }

    public double getPosition() {
        return m_motor.getPosition().getValueAsDouble();
    }

    public double getRotationValueFromTx(double tx) {
        // Removed the .isEmpty() check. It will now safely pull from the map!
        return shotMap.get(tx);
    }

    @Override
    public void periodic(){
        double currentPos = m_motor.getPosition().getValueAsDouble();

        //Helpful for calibration - you can view this in your driver station console or push to SmartDashboard
        System.out.println("GOOBA:" + currentPos);
    }
}