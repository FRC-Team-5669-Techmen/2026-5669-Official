package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Gooba = the shooter hood/arc. Kraken X44 under Motion Magic position control,
 * with an interpolating shot map from Limelight angle to hood rotations.
 */
public class GoobaSubsystem extends SubsystemBase {
    private final TalonFX m_motor = new TalonFX(Constants.Gooba.kMotorId);

    // Calibrated shot map: Limelight tx (vertical angle, sideways mount) -> hood rotations
    private final InterpolatingDoubleTreeMap shotMap = new InterpolatingDoubleTreeMap();

    // Motion Magic Request (smooth position control)
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

        // Current Limits (essential for Kraken X44 safety)
        configs.CurrentLimits.SupplyCurrentLimit = Constants.Gooba.kSupplyCurrentLimit;
        configs.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Firmware soft limits: hood travel is 0.0 to 10.7 rotations
        configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
        configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 10.7;
        configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        m_motor.getConfigurator().apply(configs);

        m_motor.setNeutralMode(NeutralModeValue.Brake);
        m_motor.setPosition(0);

        // ==========================================
        // SHOT MAP CALIBRATION DATA
        // Format: shotMap.put(limelight_tx_degrees, hood_position_rotations);
        // Re-record these points whenever the hood or camera mount changes.
        // ==========================================
        shotMap.put(27.3, 1.0);
        shotMap.put(-16.8, 2.36);
        shotMap.put(-18.3, 3.8);
        shotMap.put(-23.9, 5.1);
    }

    /** Motion Magic move, kept inside the 0.0 to 10.7 rotation soft limits. */
    public void setPosition(double rotations) {
        m_motor.setControl(m_positionControl.withPosition(rotations));
    }

    public double getPosition() {
        return m_motor.getPosition().getValueAsDouble();
    }

    /** Interpolates the calibrated shot map for the hood position matching this tx. */
    public double getRotationValueFromTx(double tx) {
        return shotMap.get(tx);
    }
}
