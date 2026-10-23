package frc.robot.subsystems;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GroundIntakePivot;

/**
 * Raises and lowers the ground intake. Two motors work as a pair: motor 20 (the
 * leader) is the only one the code ever commands, and motor 21 copies it.
 *
 * <p>Driver A toggles: first press DOWN, next press UP, and so on.
 * <ul>
 *   <li>DOWN: Motion Magic down smoothly. Once it gets there, or the motors feel it
 *       resting on the floor (a "stall"), it stops pushing and rests in brake mode.</li>
 *   <li>UP: Motion Magic up smoothly, then a gentle push until the motors feel the top
 *       hard stop. That spot becomes UP = 0, so a wrong zero fixes itself.</li>
 * </ul>
 * Operator D-pad Up/Down jogs it slowly by hand (manual override).
 *
 * <p>Positions are in motor rotations, and POSITIVE = UP. DOWN is a negative number.
 */
public class GroundIntakePivotSubsystem extends SubsystemBase {
    private final TalonFX leader = new TalonFX(GroundIntakePivot.kLeaderId);
    private final TalonFX follower = new TalonFX(GroundIntakePivot.kFollowerId);

    // FOC must be OFF on every command. These motors have no Phoenix Pro license, and
    // an unlicensed motor refuses FOC commands (that's why motor 20 sat still before).
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(false);
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(false);

    private boolean isDown = false;    // every match starts UP
    private boolean isHolding = false; // true while Motion Magic is holding a target
    private double targetPosition = GroundIntakePivot.kUpPosition;

    // "Stalled" = pushing hard but not moving, for a moment. Updated every loop in periodic().
    private final Debouncer stallDebouncer =
        new Debouncer(GroundIntakePivot.kStallSeconds, Debouncer.DebounceType.kRising);
    private boolean stalled = false;

    // DOWN soft limit ON normally, everything OFF only while the Operator D-pad jog is held.
    private final SoftwareLimitSwitchConfigs softLimitsOn;
    private final SoftwareLimitSwitchConfigs softLimitsOff = new SoftwareLimitSwitchConfigs(); // all off

    public GroundIntakePivotSubsystem() {
        // Follower gets the shared settings only. No soft limits on the follower: its own
        // position counts the opposite way (it's mirrored), so they'd stop it at the wrong time.
        follower.getConfigurator().apply(sharedConfig());

        // Leader gets the shared settings + Motion Magic + the DOWN soft limit.
        TalonFXConfiguration leaderConfig = sharedConfig();
        leaderConfig.Slot0.kP = GroundIntakePivot.kP;
        leaderConfig.Slot0.kD = GroundIntakePivot.kD;
        leaderConfig.Slot0.kV = GroundIntakePivot.kV;
        leaderConfig.MotionMagic.MotionMagicCruiseVelocity = GroundIntakePivot.kCruiseVelocity;
        leaderConfig.MotionMagic.MotionMagicAcceleration = GroundIntakePivot.kAcceleration;
        leaderConfig.MotionMagic.MotionMagicJerk = GroundIntakePivot.kJerk;

        // No UP soft limit on purpose: the top hard stop is how the code finds UP (see the
        // UP move below). A soft limit there, measured from a wrong zero, is what blocked
        // raising before. DOWN soft limit: just past DOWN, once DOWN is set.
        if (isDownPositionValid()) {
            leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                GroundIntakePivot.kDownPosition - GroundIntakePivot.kSoftLimitMargin;
            leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        }
        leader.getConfigurator().apply(leaderConfig);
        softLimitsOn = leaderConfig.SoftwareLimitSwitch;

        // NO zeroing here on purpose: deploying code must not re-zero wherever the intake
        // happens to be. The first UP press finds the true top by itself.

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

    /** Where the A button sends it for DOWN: a little short of kDownPosition. */
    private static double downTarget() {
        return GroundIntakePivot.kDownPosition + GroundIntakePivot.kDownBackoff;
    }

    /** DOWN is usable only once it's set, meaning it's clearly below UP. */
    private static boolean isDownPositionValid() {
        return downTarget() < GroundIntakePivot.kUpPosition;
    }

    private double position() {
        return leader.getPosition().getValueAsDouble();
    }

    private boolean isNear(double target) {
        return Math.abs(position() - target) < GroundIntakePivot.kAtTargetTolerance;
    }

    /** Forget any old stall so a new move starts fresh. */
    private void resetStall() {
        stalled = false;
        stallDebouncer.calculate(false);
    }

    /** Motion Magic to a position. The leader keeps holding it there until told otherwise. */
    private void goTo(double position) {
        resetStall();
        targetPosition = position;
        isHolding = true;
        leader.setControl(positionRequest.withPosition(position));
    }

    /** Push at a fixed voltage. Positive = up. Cancels any Motion Magic target. */
    private void setVolts(double volts) {
        resetStall();
        isHolding = false;
        leader.setControl(voltageRequest.withOutput(volts));
    }

    /** Stops both motors (brake mode). The follower copies the leader, so it stops too. */
    public void stop() {
        isHolding = false;
        leader.stopMotor();
    }

    /** Where UP is held: a hair below the top hard stop, so the motors aren't pressing into it. */
    private static double upHoldTarget() {
        return GroundIntakePivot.kUpPosition - GroundIntakePivot.kUpHoldOffset;
    }

    /** UP finished: if it really pushed against the top hard stop, call that spot UP = 0. Then hold UP. */
    private void finishUp() {
        if (stalled) {
            leader.setPosition(GroundIntakePivot.kUpPosition);
            follower.setPosition(0);
        }
        goTo(upHoldTarget());
    }

    /** DOWN: move down, then stop pushing once it's there or resting on the floor. */
    private Command downMove() {
        return Commands.sequence(
            runOnce(() -> goTo(downTarget())),
            Commands.waitUntil(() -> stalled || isNear(downTarget()))
                .withTimeout(GroundIntakePivot.kMoveTimeout),
            runOnce(this::stop)); // rest in brake mode: no pushing, no humming
    }

    /** UP: move up, then push gently until it feels the top hard stop, re-zero there, hold. */
    private Command upMove() {
        return Commands.sequence(
            runOnce(() -> goTo(upHoldTarget())),
            Commands.waitUntil(() -> stalled || isNear(upHoldTarget()))
                .withTimeout(GroundIntakePivot.kMoveTimeout),
            runOnce(() -> setVolts(GroundIntakePivot.kSeatVolts)),
            Commands.waitUntil(() -> stalled)
                .withTimeout(GroundIntakePivot.kSeatTimeout),
            runOnce(this::finishUp));
    }

    /**
     * Driver A: first press DOWN, next press UP, and so on. Does nothing until DOWN is set.
     * This command is instant on purpose: it flips the toggle and starts a fresh move, which
     * cancels any move still running. That way every press counts, even mid-move.
     */
    public Command toggleCommand() {
        return Commands.runOnce(() -> {
            if (!isDownPositionValid()) {
                return; // the dashboard "Intake Status" says why
            }
            isDown = !isDown; // remember the press, go the other way this time
            CommandScheduler.getInstance().schedule(isDown ? downMove() : upMove());
        });
    }

    /** Operator D-pad Up: hold to jog up slowly, release to stop. */
    public Command jogUpCommand() {
        return startEnd(() -> startJog(GroundIntakePivot.kJogVolts), this::endJog);
    }

    /** Operator D-pad Down: hold to jog down slowly, release to stop. */
    public Command jogDownCommand() {
        return startEnd(() -> startJog(-GroundIntakePivot.kJogVolts), this::endJog);
    }

    /**
     * The jog is the manual override: it ignores the soft limits so it can ALWAYS bring
     * the intake back, even if the position numbers are off. It's slow, and letting go stops it.
     */
    private void startJog(double volts) {
        leader.getConfigurator().apply(softLimitsOff);
        setVolts(volts);
    }

    private void endJog() {
        stop();
        leader.getConfigurator().apply(softLimitsOn); // DOWN soft limit back on
    }

    /** Runs when the robot disables, so re-enabling can never move the intake by itself. */
    public Command stopCommand() {
        return runOnce(this::stop).ignoringDisable(true);
    }

    /** The "Zero Intake (intake is UP)" dashboard button. Works while disabled. */
    public Command zeroCommand() {
        return runOnce(() -> {
            stop();
            leader.setPosition(GroundIntakePivot.kUpPosition);
            follower.setPosition(0);
            isDown = false;
            targetPosition = GroundIntakePivot.kUpPosition;
        }).ignoringDisable(true).withName("Zero Intake");
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
        double speed = leader.getVelocity().getValueAsDouble();
        double amps = leader.getStatorCurrent().getValueAsDouble();
        stalled = stallDebouncer.calculate(
            Math.abs(speed) < GroundIntakePivot.kStallSpeed
                && Math.abs(amps) > GroundIntakePivot.kStallAmps);

        SmartDashboard.putString("Intake Status", status());
        SmartDashboard.putString("Intake Next A Press", isDown ? "goes UP" : "goes DOWN");
        SmartDashboard.putNumber("Intake Position (rot)", position());
        // Left side (follower). It may count the opposite way, so compare the SIZE with
        // "Intake Position": if the right says -4.3 and the left says 3.8, the left is
        // about 0.5 rotations behind.
        SmartDashboard.putNumber("Intake Left Side Position (rot)", follower.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Intake Target (rot)", targetPosition);
        SmartDashboard.putBoolean("Intake Holding Target", isHolding);
        SmartDashboard.putBoolean("Intake Stalled", stalled);
        // If both amps climb high but it doesn't move, the motors are fighting: flip kFollowerAlignment.
        SmartDashboard.putNumber("Intake Pivot Leader Amps", amps);
        SmartDashboard.putNumber("Intake Pivot Follower Amps", follower.getStatorCurrent().getValueAsDouble());
        // While moving, both speeds should be about the same size with OPPOSITE signs.
        SmartDashboard.putNumber("Intake Pivot Leader Speed (rot/s)", speed);
        SmartDashboard.putNumber("Intake Pivot Follower Speed (rot/s)", follower.getVelocity().getValueAsDouble());
    }
}
