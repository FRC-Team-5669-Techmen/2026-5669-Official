package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GroundIntakePivot;

/**
 * Raises and lowers the ground intake. Two motors work as a pair: motor 20 (the
 * leader) is the only one the code ever commands, and motor 21 copies it.
 *
 * <p>Driver A toggles UP/DOWN with Motion Magic, a position mode that speeds up and
 * slows down smoothly instead of slamming into the ends. Operator D-pad Up/Down jogs
 * it slowly by hand.
 *
 * <p>Positions are in motor rotations, and POSITIVE = UP. UP is 0, set at startup
 * because every match starts with the intake up. DOWN is a negative number you measure.
 */
public class GroundIntakePivotSubsystem extends SubsystemBase {
    private final TalonFX leader = new TalonFX(GroundIntakePivot.kLeaderId);
    private final TalonFX follower = new TalonFX(GroundIntakePivot.kFollowerId);

    // FOC must be OFF on every command. These motors have no Phoenix Pro license, and
    // an unlicensed motor refuses FOC commands (that's why motor 20 sat still before).
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(false);
    private final VoltageOut jogRequest = new VoltageOut(0).withEnableFOC(false);

    private boolean isDown = false;    // every match starts UP
    private boolean isHolding = false; // true while Motion Magic is holding a target
    private double targetPosition = GroundIntakePivot.kUpPosition;

    public GroundIntakePivotSubsystem() {
        // Follower gets the shared settings only. No soft limits on the follower: its own
        // position counts the opposite way (it's mirrored), so they'd stop it at the wrong time.
        follower.getConfigurator().apply(sharedConfig());

        // Leader gets the shared settings + Motion Magic + soft limits.
        TalonFXConfiguration leaderConfig = sharedConfig();
        leaderConfig.Slot0.kP = GroundIntakePivot.kP;
        leaderConfig.Slot0.kD = GroundIntakePivot.kD;
        leaderConfig.Slot0.kV = GroundIntakePivot.kV;
        leaderConfig.MotionMagic.MotionMagicCruiseVelocity = GroundIntakePivot.kCruiseVelocity;
        leaderConfig.MotionMagic.MotionMagicAcceleration = GroundIntakePivot.kAcceleration;
        leaderConfig.MotionMagic.MotionMagicJerk = GroundIntakePivot.kJerk;

        // Soft limits: the motor refuses to drive past these, no matter what it's told.
        // Just past UP:
        leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
            GroundIntakePivot.kUpPosition + GroundIntakePivot.kSoftLimitMargin;
        leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // Just past DOWN, but only once DOWN has been measured. Until then this limit is
        // off, so you can jog all the way down to measure it.
        if (isDownPositionValid()) {
            leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                GroundIntakePivot.kDownPosition - GroundIntakePivot.kSoftLimitMargin;
            leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        }
        leader.getConfigurator().apply(leaderConfig);

        // Every match starts with the intake UP: call this spot UP = 0. The follower is
        // zeroed too, only so the dashboard can show how far the left side lags.
        leader.setPosition(GroundIntakePivot.kUpPosition);
        follower.setPosition(0);

        // Tell motor 21 to copy motor 20. Opposed = spin the opposite way (mirrored mount).
        follower.setControl(new Follower(GroundIntakePivot.kLeaderId, GroundIntakePivot.kFollowerAlignment));
    }

    /** Settings both motors share: brake mode, direction, and the stator current limit. */
    private static TalonFXConfiguration sharedConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = GroundIntakePivot.kLeaderInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.StatorCurrentLimit = GroundIntakePivot.kStatorCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        return config;
    }

    /** Where the A button sends it for DOWN: a little short of the measured ground spot. */
    private static double downTarget() {
        return GroundIntakePivot.kDownPosition + GroundIntakePivot.kDownBackoff;
    }

    /** DOWN is usable only once it's been measured, meaning it's clearly below UP. */
    private static boolean isDownPositionValid() {
        return downTarget() < GroundIntakePivot.kUpPosition;
    }

    /** Driver A: first press goes DOWN, the next goes UP, and so on. Does nothing until DOWN is set. */
    public void toggleUpDown() {
        if (!isDownPositionValid()) {
            return; // the dashboard "Intake Status" says why
        }
        isDown = !isDown;
        goTo(isDown ? downTarget() : GroundIntakePivot.kUpPosition);
    }

    /** Motion Magic to a position. The leader keeps holding it there until told otherwise. */
    private void goTo(double position) {
        targetPosition = position;
        isHolding = true;
        leader.setControl(positionRequest.withPosition(position));
    }

    /** Slow manual move. Positive volts = up. Cancels any Motion Magic target. */
    public void jog(double volts) {
        isHolding = false;
        leader.setControl(jogRequest.withOutput(volts));
    }

    /** Stops both motors (brake mode). The follower copies the leader, so it stops too. */
    public void stop() {
        isHolding = false;
        leader.stopMotor();
    }

    /** Only use when the intake is physically UP: resets UP = 0 and the toggle back to UP. */
    public void zeroAtUp() {
        stop();
        leader.setPosition(GroundIntakePivot.kUpPosition);
        follower.setPosition(0);
        isDown = false;
        targetPosition = GroundIntakePivot.kUpPosition;
    }

    /** Driver A. */
    public Command toggleCommand() {
        return runOnce(this::toggleUpDown);
    }

    /** Operator D-pad Up: hold to jog up slowly, release to stop. */
    public Command jogUpCommand() {
        return startEnd(() -> jog(GroundIntakePivot.kJogVolts), this::stop);
    }

    /** Operator D-pad Down: hold to jog down slowly, release to stop. */
    public Command jogDownCommand() {
        return startEnd(() -> jog(-GroundIntakePivot.kJogVolts), this::stop);
    }

    /** Runs when the robot disables, so re-enabling can never move the intake by itself. */
    public Command stopCommand() {
        return runOnce(this::stop).ignoringDisable(true);
    }

    /** The "Zero Intake (intake is UP)" dashboard button. Works while disabled. */
    public Command zeroCommand() {
        return runOnce(this::zeroAtUp).ignoringDisable(true).withName("Zero Intake");
    }

    private String status() {
        if (GroundIntakePivot.kDownPosition == 0.0) {
            return "Intake DOWN position not set";
        }
        if (!isDownPositionValid()) {
            return "Intake DOWN position must be a negative number below UP";
        }
        return isDown ? "DOWN" : "UP";
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Intake Status", status());
        SmartDashboard.putNumber("Intake Position (rot)", leader.getPosition().getValueAsDouble());
        // Left side (follower). It may count the opposite way, so compare the SIZE with
        // "Intake Position": if the right says -4.3 and the left says 3.8, the left is
        // about 0.5 rotations behind.
        SmartDashboard.putNumber("Intake Left Side Position (rot)", follower.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Intake Target (rot)", targetPosition);
        SmartDashboard.putBoolean("Intake Holding Target", isHolding);
        // If both amps climb high but it doesn't move, the motors are fighting: flip kFollowerAlignment.
        SmartDashboard.putNumber("Intake Pivot Leader Amps", leader.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake Pivot Follower Amps", follower.getStatorCurrent().getValueAsDouble());
        // While moving, both speeds should be about the same size with OPPOSITE signs.
        SmartDashboard.putNumber("Intake Pivot Leader Speed (rot/s)", leader.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake Pivot Follower Speed (rot/s)", follower.getVelocity().getValueAsDouble());
    }
}
