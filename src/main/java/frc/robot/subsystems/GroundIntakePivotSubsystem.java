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
 *   <li>DOWN: Motion Magic down smoothly to just above the floor (kDownBackoff). Once it
 *       gets there (or stalls on something), it stops pushing and rests on the floor.</li>
 *   <li>UP: Motion Magic up smoothly to a little below the top (kUpHoldOffset), and KEEPS
 *       HOLDING it there so it can't drop mid-match. It never touches the top hard stop.</li>
 * </ul>
 * Operator D-pad Up/Down jogs it slowly by hand (manual override).
 *
 * <p>Positions are in motor rotations, and POSITIVE = UP. DOWN is a negative number.
 * The motors count from where the intake was at power-on (no sensor knows the real angle),
 * so the intake MUST be pushed all the way up before power-on. If it wasn't, the first
 * DOWN stops short of the floor and "Intake Status" says ZERO WRONG.
 */
public class GroundIntakePivotSubsystem extends SubsystemBase {
    private final TalonFX leader = new TalonFX(GroundIntakePivot.kLeaderId);
    private final TalonFX follower = new TalonFX(GroundIntakePivot.kFollowerId);

    // FOC must be OFF on every command. These motors have no Phoenix Pro license, and
    // an unlicensed motor refuses FOC commands (that's why motor 20 sat still before).
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(false);
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(false);

    private boolean isDown = false;    // set in the constructor from where the intake really is
    private boolean isHolding = false; // true while Motion Magic is holding a target
    private double targetPosition = GroundIntakePivot.kUpPosition;
    // How far short of the floor the last DOWN stopped. Big = UP = 0 is wrong. See downMove().
    private double zeroError = 0.0;

    // "Stalled" = pushing hard but not moving, for a moment. Updated every loop in periodic().
    private final Debouncer stallDebouncer =
        new Debouncer(GroundIntakePivot.kStallSeconds, Debouncer.DebounceType.kRising);
    private boolean stalled = false;
    // Oct 28 safety: going UP but stuck against something well short of UP (the top hard stop
    // reached early = the zero is wrong, e.g. "intakeStartsDown" ran with the intake really UP).
    // It stops pushing instead of grinding at 40 A until somebody notices.
    private boolean upBlocked = false;
    private double leaderSpeed = 0.0; // rot/s, updated every loop in periodic()

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

        // No UP soft limit on purpose: one measured from a wrong zero is what blocked
        // raising before. DOWN soft limit: just past DOWN, once DOWN is set.
        if (isDownPositionValid()) {
            leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                GroundIntakePivot.kDownPosition - GroundIntakePivot.kSoftLimitMargin;
            leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        }
        leader.getConfigurator().apply(leaderConfig);
        softLimitsOn = leaderConfig.SoftwareLimitSwitch;

        // NO zeroing here on purpose: deploying code must not re-zero wherever the intake
        // happens to be. UP = 0 comes from powering on with the intake up (or the
        // "Zero Intake" dashboard button).

        // But DO check where it is right now: below halfway counts as DOWN, so after a code
        // redeploy with the intake down, the first A press raises it. (This only helps if
        // the robot stayed powered on. At power-on the motors read 0, which looks like UP.)
        double startPosition = leader.getPosition().waitForUpdate(0.25).getValueAsDouble();
        isDown = startPosition < (GroundIntakePivot.kUpPosition + GroundIntakePivot.kDownPosition) / 2;

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
        upBlocked = false;
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

    /** Where the A button sends it for UP: a little below the top hard stop, so it never touches it. */
    private static double upTarget() {
        return GroundIntakePivot.kUpPosition - GroundIntakePivot.kUpHoldOffset;
    }

    /**
     * DOWN: Motion Magic down, then stop pushing once it has arrived and rest on the floor
     * in brake mode (gravity keeps it there, no humming).
     */
    private Command downMove() {
        return Commands.sequence(
            runOnce(() -> goTo(downTarget())),
            // Keep driving until it has actually ARRIVED (close AND stopped moving), so the
            // motors control the whole way. Stopping early let it free-fall, and each side
            // fell at its own speed (that was the "out of sync"). If it stalls against
            // something, it stops right away instead of pushing.
            Commands.waitUntil(() -> stalled
                    || (isNear(downTarget()) && Math.abs(leaderSpeed) < GroundIntakePivot.kStallSpeed))
                .withTimeout(GroundIntakePivot.kMoveTimeout),
            runOnce(() -> {
                // Stopped well short of the floor? Then UP = 0 is wrong (it wasn't all the way
                // up at power-on), and every position is off by this much. Elastic says so.
                zeroError = position() - downTarget();
                stop();
            }));
    }

    /**
     * UP: Motion Magic up to a little below the top, then KEEP HOLDING it there until the
     * next press, so it can't drop mid-match while the robot moves around. Holding takes a
     * little power, so a faint hum at UP is normal.
     */
    private Command upMove() {
        return runOnce(() -> goTo(upTarget()));
    }

    private boolean isZeroWrong() {
        return zeroError > GroundIntakePivot.kZeroWrongTolerance;
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

    /** Vacuum mode: put the intake DOWN if it isn't already (same move as the A button's DOWN). */
    /** The intake is up (or going up): the last toggle/command sent it UP. */
    public boolean isUpish() { return !isDown; }

    public void deployIfUp() {
        if (!isDownPositionValid() || isDown) return;
        isDown = true;
        CommandScheduler.getInstance().schedule(downMove());
    }

    /** Autos ("raiseIntake") and defense mode: put the intake UP if it's down (same move as the A button's UP). */
    public void stowIfDown() {
        if (!isDownPositionValid() || !isDown) return;
        isDown = false;
        CommandScheduler.getInstance().schedule(upMove());
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
        leader.getConfigurator().apply(softLimitsOff, 0.0); // 0 = don't wait for the reply (no loop stall)
        setVolts(volts);
    }

    private void endJog() {
        stop();
        // Driver A's toggle follows where the jog left it (it used to need a second press).
        isDown = position() < (GroundIntakePivot.kUpPosition + GroundIntakePivot.kDownPosition) / 2;
        leader.getConfigurator().apply(softLimitsOn, 0.0); // DOWN soft limit back on (no wait)
    }

    /** Hotter of the two pivot motors, deg C. */
    public double tempCelsius() {
        return Math.max(leader.getDeviceTemp().getValueAsDouble(), follower.getDeviceTemp().getValueAsDouble());
    }

    /** True when the intake is deployed (the last A press sent it DOWN). */
    public boolean isDown() {
        return isDown && isDownPositionValid();
    }

    /**
     * AGITATION PUMP (Agitator, while shooting): lift the deployed intake part way and drop it
     * back, `cycles` times, to tap a bridged pile loose. Ends resting DOWN like the A button
     * (brake, no pushing). Any A press or jog interrupts it (the driver always wins).
     */
    public Command pumpCommand(double lift, int cycles, double legTimeout) {
        double top = Math.min(downTarget() + lift, upTarget());
        Command all = pumpCycle(top, legTimeout);
        for (int i = 1; i < cycles; i++) {
            all = all.andThen(pumpCycle(top, legTimeout));
        }
        return all.finallyDo(this::stop).withName("Agitate: pump intake");
    }

    /** One pump: up to `top` (or as far as it gets), then back down to resting DOWN. */
    private Command pumpCycle(double top, double legTimeout) {
        return Commands.sequence(
            runOnce(() -> goTo(top)),
            Commands.waitUntil(() -> isNear(top) || stalled).withTimeout(legTimeout),
            runOnce(() -> goTo(downTarget())),
            Commands.waitUntil(() -> stalled
                    || (isNear(downTarget()) && Math.abs(leaderSpeed) < GroundIntakePivot.kStallSpeed))
                .withTimeout(legTimeout));
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
            zeroError = 0.0;
            upBlocked = false;
        }).ignoringDisable(true).withName("Zero Intake");
    }

    /**
     * The intake is DOWN on the floor right now: make that the zero. For starting a match with the
     * intake already down (the "intakeStartsDown" auto step and the "Zero Intake (intake is DOWN)"
     * button). Powering on with it down reads 0, which looks like UP: wrong by the whole travel.
     */
    public void zeroAtDown() {
        if (!isDownPositionValid()) return;
        stop();
        leader.setPosition(GroundIntakePivot.kDownPosition);
        follower.setPosition(-GroundIntakePivot.kDownPosition); // mirrored mount counts the other way
        isDown = true;
        targetPosition = GroundIntakePivot.kDownPosition;
        zeroError = 0.0;
        upBlocked = false;
    }

    /** The "Zero Intake (intake is DOWN)" dashboard button. Works while disabled. */
    public Command zeroDownCommand() {
        return runOnce(this::zeroAtDown).ignoringDisable(true).withName("Zero Intake (DOWN)");
    }

    private String status() {
        if (GroundIntakePivot.kDownPosition == 0.0) {
            return "Intake DOWN position not set";
        }
        if (!isDownPositionValid()) {
            return "Intake DOWN position must be a negative number below UP";
        }
        if (upBlocked) {
            return String.format("UP BLOCKED at %.1f: zero wrong? push intake up, press Zero Intake (intake is UP)", position());
        }
        if (isZeroWrong()) {
            return String.format("ZERO WRONG by %.1f: push intake up, press Zero Intake", zeroError);
        }
        return isDown ? "DOWN" : "UP";
    }

    @Override
    public void periodic() {
        double speed = leader.getVelocity().getValueAsDouble();
        leaderSpeed = speed;
        double amps = leader.getStatorCurrent().getValueAsDouble();
        stalled = stallDebouncer.calculate(
            Math.abs(speed) < GroundIntakePivot.kStallSpeed
                && Math.abs(amps) > GroundIntakePivot.kStallAmps);

        if (!isDown && isHolding && !upBlocked && stalled
                && targetPosition >= upTarget() - 0.01 && position() < upTarget() - 1.0) {
            stop();
            upBlocked = true;
            edu.wpi.first.wpilibj.DriverStation.reportWarning(String.format(
                "INTAKE | UP blocked at %.1f rot (should reach %.1f): stopped pushing. Zero is probably wrong:"
                + " push the intake up and press Zero Intake (intake is UP)", position(), upTarget()), false);
        }

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
