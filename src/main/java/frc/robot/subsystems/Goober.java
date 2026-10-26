package frc.robot.subsystems;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Aim;
import frc.robot.Constants.Turret;

/**
 * Goober = the turret (Talon FX 14). Turns the shooter, and the Limelight on it, toward the hub.
 *
 * <p>Positive = turning RIGHT = away from HOME. HOME (0) is one end of the ~270° travel and
 * the far end is the other. Past either end there's nothing but the wires running up to the
 * turret, so the motor's soft limits stop it at both ends.
 *
 * <p>The far end is set from the dashboard: jog to it, press "Set Turret Far End". It's saved on
 * the roboRIO (WPILib Preferences), so it survives power-offs and deploys.
 * {@link Turret#kEndPosition} is only used if nothing is saved.
 */
public class Goober extends SubsystemBase {
    private final TalonFX motor = new TalonFX(Turret.kMotorId);
    // FOC off: this motor has no Phoenix Pro license, and an unlicensed motor refuses FOC commands.
    private final DutyCycleOut request = new DutyCycleOut(0).withEnableFOC(false);

    private final PIDController turnPID = new PIDController(Turret.kP, Turret.kI, Turret.kD);

    // JITTER FIX: A moving average filter to smooth out Limelight pixel noise
    private final LinearFilter txFilter = LinearFilter.movingAverage(5);

    private String aimStatus = "OFF"; // what auto-aim is doing, for the dashboard

    // Saved on the roboRIO under this name (Elastic shows it under "Preferences").
    private static final String kEndPreferenceKey = "Turret End Position (rot)";
    // Far end in motor rotations from HOME. 0 = not set: no far-end limit, auto-aim off.
    private double endPosition = Preferences.getDouble(kEndPreferenceKey, 0.0) > 0.0
        ? Preferences.getDouble(kEndPreferenceKey, 0.0)
        : Turret.kEndPosition;

    public Goober() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.MotorOutput.Inverted = Turret.kMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        configs.CurrentLimits.StatorCurrentLimit = Turret.kStatorCurrentLimit;
        configs.CurrentLimits.StatorCurrentLimitEnable = true;

        configs.SoftwareLimitSwitch = limitConfigs();

        motor.getConfigurator().apply(configs);
        turnPID.setTolerance(Turret.kToleranceDegrees);

        // NO zeroing here on purpose: deploying code must not re-zero wherever the turret happens
        // to be, or the limits would end up in the wrong place. 0 comes from powering on with the
        // turret on the HOME mark (or the "Zero Turret" dashboard button).
    }

    /** True once the far end is set. Until then auto-aim does nothing. */
    public boolean endMeasured() {
        return endPosition > 0.0;
    }

    /** Soft limits: HOME side (0) always, far side only once it's set. */
    private SoftwareLimitSwitchConfigs limitConfigs() {
        SoftwareLimitSwitchConfigs limits = new SoftwareLimitSwitchConfigs();
        // HOME end: it can never turn left past HOME.
        limits.ReverseSoftLimitThreshold = 0.0;
        limits.ReverseSoftLimitEnable = true;
        // Far end (kEndDegrees from HOME): it can never turn right past it.
        limits.ForwardSoftLimitThreshold = endPosition;
        limits.ForwardSoftLimitEnable = endMeasured();
        return limits;
    }

    /** Motor rotations per degree the turret turns. */
    private double rotationsPerDegree() {
        return endPosition / Turret.kEndDegrees;
    }

    /** Lowest position auto-aim and the search sweep use (kAimMarginDegrees right of HOME). */
    public double aimMinPosition() {
        return Turret.kAimMarginDegrees * rotationsPerDegree();
    }

    /** Highest position auto-aim and the search sweep use (kAimMarginDegrees short of the far end). */
    public double aimMaxPosition() {
        return endPosition - aimMinPosition();
    }

    /** Percent power, positive = turn RIGHT (away from HOME). */
    public void setMotorSpeed(double percent) {
        motor.setControl(request.withOutput(percent * Turret.kSpeedMultiplier));
    }

    /**
     * One step of Limelight aiming. tx = degrees to the tag, positive = tag to the RIGHT.
     * Positive output also turns RIGHT, so the turret turns toward the tag.
     */
    public void aimAtTarget(double tx) {
        // Run the raw Limelight data through our filter to smooth the noise
        double smoothedTx = txFilter.calculate(tx);

        // The PID returns kP * (0 - tx), which is negative for a tag on the right.
        // Flip it so a tag on the RIGHT turns the turret RIGHT.
        double output = -turnPID.calculate(smoothedTx, 0.0);

        // JITTER FIX: If we are close enough to the target, cut power completely
        // so the motor stops fighting tiny gearbox vibrations.
        if (turnPID.atSetpoint()) {
            stop();
            aimStatus = "LOCKED";
            return;
        }

        // The tag is further around than the turret can turn: don't push into the end.
        double position = getPosition();
        if ((output > 0 && position >= aimMaxPosition()) || (output < 0 && position <= aimMinPosition())) {
            stop();
            aimStatus = "TAG OUT OF REACH";
            return;
        }

        setMotorSpeed(MathUtil.clamp(output, -Turret.kMaxOutput, Turret.kMaxOutput));
        aimStatus = "TRACKING";
    }

    // ---- Aiming at an ANGLE (shoot from anywhere) ----
    private final PIDController anglePID = new PIDController(Aim.kTurretP, 0.0, Aim.kTurretD);
    private double angleTarget = Double.NaN; // degrees from HOME, NaN = not aiming at an angle

    /**
     * One step of turning the turret to an angle (degrees clockwise from HOME), for shooting
     * from anywhere. robotSpinRadPerSec is the robot's own rotation (CCW positive): the turret
     * turns the other way to cancel it, so it stays on the hub while the robot spins.
     * Clamped inside the aim range. Does nothing until the far end is set.
     */
    public void aimAtAngle(double targetDegrees, double robotSpinRadPerSec) {
        if (!endMeasured()) {
            stop();
            aimStatus = "OFF: far end not set";
            return;
        }
        double lo = Turret.kAimMarginDegrees;
        double hi = Turret.kEndDegrees - Turret.kAimMarginDegrees;
        angleTarget = MathUtil.clamp(targetDegrees, lo, hi);
        double angle = getAngleDegrees();
        // Robot turning CCW at w makes the needed turret angle (clockwise from HOME) grow at w.
        double feedforward = Aim.kTurretFF * Math.toDegrees(robotSpinRadPerSec);
        double error = angleTarget - angle;
        double output = anglePID.calculate(angle, angleTarget) + feedforward;
        // Static-friction kick so small errors still move the turret (no kick once close).
        if (Math.abs(error) > Aim.kTurretToleranceDegrees * 0.5) {
            output += Math.copySign(Aim.kTurretKS, error);
        }
        SmartDashboard.putNumber("Turret Aim Error (deg)", -error);
        setMotorSpeed(MathUtil.clamp(output, -Aim.kTurretMaxOutput, Aim.kTurretMaxOutput));
        aimStatus = aimLocked() ? "LOCKED" : "TURNING";
    }

    /** On the angle target and nearly still. */
    public boolean aimLocked() {
        return !Double.isNaN(angleTarget)
            && Math.abs(getAngleDegrees() - angleTarget) <= Aim.kTurretToleranceDegrees
            && Math.abs(getVelocityDegPerSec()) <= Aim.kLockedMaxSpeedDegPerSec;
    }

    /** Degrees between the turret and its angle target (0 when not aiming at an angle). */
    public double aimErrorDegrees() {
        return Double.isNaN(angleTarget) ? 0.0 : getAngleDegrees() - angleTarget;
    }

    /** Forget the angle target (aiming stopped). */
    public void clearAngleTarget() {
        angleTarget = Double.NaN;
        anglePID.reset();
    }

    /** Turret speed, degrees per second, positive = turning RIGHT. 0 until the far end is set. */
    public double getVelocityDegPerSec() {
        return endMeasured() ? motor.getVelocity().getValueAsDouble() / rotationsPerDegree() : 0.0;
    }

    /** Clears the tx smoothing filter so a new alignment doesn't chase stale samples. */
    public void resetAimFilter() {
        txFilter.reset();
    }

    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    /** Degrees from HOME (0 = HOME, kEndDegrees = far end). Reads 0 until the far end is set. */
    public double getAngleDegrees() {
        return endMeasured() ? getPosition() / rotationsPerDegree() : 0.0;
    }

    public void stop() {
        motor.stopMotor();
    }

    public boolean targetLocked() {
        return turnPID.atSetpoint();
    }

    /** For GooberAlign: what auto-aim is doing, shown as "Turret Auto-Aim". */
    public void setAimStatus(String status) {
        aimStatus = status;
    }

    /** The "Zero Turret (turret at HOME)" dashboard button: call this spot 0. Works while disabled. */
    public Command zeroCommand() {
        return runOnce(() -> motor.setPosition(0)).ignoringDisable(true).withName("Zero Turret");
    }

    /**
     * The "Set Turret Far End (turret at far end)" dashboard button: the turret's current spot
     * becomes the far end (kEndDegrees from HOME), saved on the roboRIO. Works while disabled.
     */
    public Command setEndCommand() {
        return runOnce(() -> {
            double position = getPosition();
            if (position < Turret.kMinEndPosition) {
                DriverStation.reportWarning("Turret far end NOT saved: the turret is at HOME."
                    + " Jog it Right to the far end first.", false);
                return;
            }
            endPosition = position;
            Preferences.setDouble(kEndPreferenceKey, position);
            motor.getConfigurator().apply(limitConfigs());
        }).ignoringDisable(true).withName("Set Turret Far End");
    }

    /**
     * The "Clear Turret Far End" dashboard button: forgets the saved far end (back to
     * kEndPosition, which is 0.0 = no far-end limit), so it can be measured again.
     */
    public Command clearEndCommand() {
        return runOnce(() -> {
            Preferences.setDouble(kEndPreferenceKey, 0.0);
            endPosition = Turret.kEndPosition;
            motor.getConfigurator().apply(limitConfigs());
        }).ignoringDisable(true).withName("Clear Turret Far End");
    }

    private String status() {
        if (endPosition < 0.0) {
            return "BACKWARDS: flip kMotorInverted, re-measure kEndPosition";
        }
        if (!endMeasured()) {
            return "Far end NOT set: NO 270 limit! Jog to it, press Set Turret Far End";
        }
        double position = getPosition();
        if (position <= 0.0) {
            return "At HOME end";
        }
        if (position >= endPosition) {
            return "At FAR end";
        }
        return "OK";
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Turret Status", status());
        SmartDashboard.putString("Turret Auto-Aim", aimStatus);
        SmartDashboard.putNumber("Turret Position (rot)", getPosition());
        SmartDashboard.putNumber("Turret Angle (deg)", getAngleDegrees());
        SmartDashboard.putNumber("Turret Far End (rot)", endPosition);
        SmartDashboard.putNumber("Turret Amps", motor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Turret Target (deg)", Double.isNaN(angleTarget) ? -1 : angleTarget);
    }
}
