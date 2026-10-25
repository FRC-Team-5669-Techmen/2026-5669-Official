package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Gooba;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Gooba = the shooter hood. Kraken X44 (Talon FX 18) under Motion Magic position control,
 * with an interpolating shot map from the Limelight's up/down angle to the tag (ty, degrees)
 * to hood rotations. Same code as last season, which ran on ID 51.
 *
 * <p>0 = power-on position, so the hood must be all the way DOWN at power-on. Positive = UP.
 * Soft limits keep it between 0 and {@link Gooba#kTopPosition}.
 */
public class GoobaSubsystem extends SubsystemBase {
    // Stays null while kHoodEnabled is false. Every method checks for that.
    private final TalonFX m_motor;

    // Shot map: Limelight ty (degrees) -> hood rotations
    private final InterpolatingDoubleTreeMap shotMap = new InterpolatingDoubleTreeMap();

    // Motion Magic Request (smooth position control). FOC off: no Phoenix Pro license on 18.
    private final MotionMagicVoltage m_positionControl = new MotionMagicVoltage(0).withEnableFOC(false);

    private double m_target = 0.0;       // last position sent, for the dashboard
    private boolean m_deployed = false;  // Operator Y toggles this

    public GoobaSubsystem() {
        if (!Gooba.kHoodEnabled) {
            m_motor = null;
            return;
        }
        m_motor = new TalonFX(Gooba.kMotorId);
        TalonFXConfiguration configs = new TalonFXConfiguration();

        // PID for Position
        configs.Slot0.kP = Gooba.kP;
        configs.Slot0.kI = Gooba.kI;
        configs.Slot0.kD = Gooba.kD;

        // Motion Magic parameters
        configs.MotionMagic.MotionMagicCruiseVelocity = Gooba.kCruiseVelocity;
        configs.MotionMagic.MotionMagicAcceleration = Gooba.kAcceleration;

        // Current Limits (essential for Kraken X44 safety)
        configs.CurrentLimits.SupplyCurrentLimit = Gooba.kSupplyCurrentLimit;
        configs.CurrentLimits.SupplyCurrentLimitEnable = true;
        configs.CurrentLimits.StatorCurrentLimit = Gooba.kStatorCurrentLimit;
        configs.CurrentLimits.StatorCurrentLimitEnable = true;

        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.MotorOutput.Inverted = Gooba.kInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        // Firmware soft limits: hood travel is 0.0 (down) to kTopPosition
        configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
        configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Gooba.kTopPosition;
        configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        m_motor.getConfigurator().apply(configs);

        // NO zeroing here on purpose (last season's code did): deploying code must not re-zero
        // wherever the hood happens to be. 0 comes from powering on with the hood DOWN (or the
        // "Zero Hood (hood is DOWN)" dashboard button).

        // ==========================================
        // SHOT MAP CALIBRATION DATA (last season's competition numbers)
        // Format: shotMap.put(limelight_ty_degrees, hood_position_rotations);
        // Last season the camera was mounted sideways, so these were recorded from raw tx,
        // which was the up/down angle. Same camera spot, so they carry over as ty.
        // Close to the hub = big positive ty, far = negative ty.
        // To re-tune: park, jog the hood (Operator Left Stick) until shots land, then
        // record "Limelight ty" and "Hood Position (rot)" here.
        // ==========================================
        shotMap.put(27.3, 1.0);
        shotMap.put(-16.8, 2.36);
        shotMap.put(-18.3, 3.8);
        shotMap.put(-23.9, 5.1);
    }

    /** Motion Magic move, kept inside the 0.0 to kTopPosition soft limits. */
    public void setPosition(double rotations) {
        if (m_motor == null) {
            return;
        }
        m_target = MathUtil.clamp(rotations, 0.0, Gooba.kTopPosition);
        m_motor.setControl(m_positionControl.withPosition(m_target));
    }

    public double getPosition() {
        return m_motor == null ? 0.0 : m_motor.getPosition().getValueAsDouble();
    }

    /**
     * Interpolates the shot map for the hood position matching this Limelight ty.
     * Angles outside the calibrated range clamp to the nearest end point.
     */
    public double getRotationForTy(double tyDegrees) {
        return shotMap.get(tyDegrees);
    }

    /** Lets the hood go (brake mode holds it). Used when the robot disables. */
    public void stop() {
        if (m_motor != null) {
            m_motor.stopMotor();
        }
    }

    /** Operator Y: toggle between the stowed and deployed presets. */
    public Command toggleCommand() {
        return runOnce(() -> {
            m_deployed = !m_deployed;
            setPosition(m_deployed ? Gooba.kPositionDeployed : Gooba.kPositionStowed);
        });
    }

    /** On disable: drop the target so the hood can't jump to an old one when re-enabled. */
    public Command stopCommand() {
        return runOnce(this::stop).ignoringDisable(true);
    }

    /** The "Zero Hood (hood is DOWN)" dashboard button: call this spot 0. Works while disabled. */
    public Command zeroCommand() {
        return runOnce(() -> {
            if (m_motor != null) {
                m_motor.setPosition(0);
            }
        }).ignoringDisable(true).withName("Zero Hood");
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Hood Enabled", m_motor != null);
        if (m_motor != null) {
            SmartDashboard.putNumber("Hood Position (rot)", getPosition());
            SmartDashboard.putNumber("Hood Target (rot)", m_target);
            SmartDashboard.putNumber("Hood Amps", m_motor.getStatorCurrent().getValueAsDouble());
        }
    }
}
