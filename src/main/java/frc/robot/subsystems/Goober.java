package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Turret;

/**
 * Goober = the turret (Talon FX 14). Turns the shooter, and the Limelight on it, toward the hub.
 *
 * <p>Positive = turning RIGHT = away from HOME. HOME (0) is one end of the ~270° travel and
 * {@link Turret#kEndPosition} is the other. Past either end there's nothing but the wires running
 * up to the turret, so the motor's soft limits stop it at both ends.
 */
public class Goober extends SubsystemBase {
    private final TalonFX motor = new TalonFX(Turret.kMotorId);
    // FOC off: this motor has no Phoenix Pro license, and an unlicensed motor refuses FOC commands.
    private final DutyCycleOut request = new DutyCycleOut(0).withEnableFOC(false);

    private final PIDController turnPID = new PIDController(Turret.kP, Turret.kI, Turret.kD);

    // JITTER FIX: A moving average filter to smooth out Limelight pixel noise
    private final LinearFilter txFilter = LinearFilter.movingAverage(5);

    private String aimStatus = "OFF"; // what auto-aim is doing, for the dashboard

    public Goober() {
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configs.MotorOutput.Inverted = Turret.kMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        configs.CurrentLimits.StatorCurrentLimit = Turret.kStatorCurrentLimit;
        configs.CurrentLimits.StatorCurrentLimitEnable = true;

        // HOME end: it can never turn left past HOME.
        configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
        configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        // Far end: only once kEndPosition is measured.
        configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Turret.kEndPosition;
        configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = endMeasured();

        motor.getConfigurator().apply(configs);
        turnPID.setTolerance(Turret.kToleranceDegrees);

        // NO zeroing here on purpose: deploying code must not re-zero wherever the turret happens
        // to be, or the limits would end up in the wrong place. 0 comes from powering on with the
        // turret on the HOME mark (or the "Zero Turret" dashboard button).
    }

    /** True once the far end (kEndPosition) is measured. Until then auto-aim does nothing. */
    public boolean endMeasured() {
        return Turret.kEndPosition > 0.0;
    }

    /** Motor rotations per degree the turret turns. */
    private double rotationsPerDegree() {
        return Turret.kEndPosition / Turret.kEndDegrees;
    }

    /** Lowest position auto-aim and the search sweep use (kAimMarginDegrees right of HOME). */
    public double aimMinPosition() {
        return Turret.kAimMarginDegrees * rotationsPerDegree();
    }

    /** Highest position auto-aim and the search sweep use (kAimMarginDegrees short of the far end). */
    public double aimMaxPosition() {
        return Turret.kEndPosition - aimMinPosition();
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

    /** Clears the tx smoothing filter so a new alignment doesn't chase stale samples. */
    public void resetAimFilter() {
        txFilter.reset();
    }

    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    /** Degrees from HOME (0 = HOME, kEndDegrees = far end). Reads 0 until kEndPosition is measured. */
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

    private String status() {
        if (Turret.kEndPosition < 0.0) {
            return "BACKWARDS: flip kMotorInverted, re-measure kEndPosition";
        }
        if (!endMeasured()) {
            return "Far end not set: jog Right to it, read Turret Position";
        }
        double position = getPosition();
        if (position <= 0.0) {
            return "At HOME end";
        }
        if (position >= Turret.kEndPosition) {
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
        SmartDashboard.putNumber("Turret Amps", motor.getStatorCurrent().getValueAsDouble());
    }
}
