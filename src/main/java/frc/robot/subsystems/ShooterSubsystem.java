package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Dual-TalonFX flywheel shooter under velocity control, with a software ramp
 * that smooths spin-down and idle spin-up while leaving shot spin-up instant.
 */
public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX leader = new TalonFX(Constants.Shooter.kLeaderId);
    private final TalonFX follower = new TalonFX(Constants.Shooter.kFollowerId);

    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);

    private double m_targetRpm = 0.0;

    // Internal software tracker to guarantee smooth ramps
    private double m_rampedSetpointRpm = 0.0;

    public ShooterSubsystem() {
        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();

        leaderConfig.Slot0.kP = Constants.Shooter.kP;
        leaderConfig.Slot0.kV = Constants.Shooter.kV;
        leaderConfig.Slot0.kS = Constants.Shooter.kS;
        leaderConfig.Slot0.kI = Constants.Shooter.kI;
        leaderConfig.Slot0.kD = Constants.Shooter.kD;

        followerConfig.Slot0.kP = Constants.Shooter.kP;
        followerConfig.Slot0.kV = Constants.Shooter.kV;
        followerConfig.Slot0.kS = Constants.Shooter.kS;
        followerConfig.Slot0.kI = Constants.Shooter.kI;
        followerConfig.Slot0.kD = Constants.Shooter.kD;

        leaderConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Shooter.kVoltageRampPeriod;
        leaderConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Shooter.kDutyCycleRampPeriod;
        followerConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Shooter.kVoltageRampPeriod;
        followerConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Shooter.kDutyCycleRampPeriod;

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
        currentLimits.SupplyCurrentLimit = Constants.Shooter.kSupplyCurrentLimit;
        currentLimits.SupplyCurrentLimitEnable = true;

        currentLimits.StatorCurrentLimit = Constants.Shooter.kStatorCurrentLimit;
        currentLimits.StatorCurrentLimitEnable = true;

        leaderConfig.CurrentLimits = currentLimits;
        followerConfig.CurrentLimits = currentLimits;

        leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        leaderConfig.MotorOutput.Inverted = Constants.Shooter.kLeaderInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        followerConfig.MotorOutput.Inverted = Constants.Shooter.kFollowerInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        leader.getConfigurator().apply(leaderConfig);
        follower.getConfigurator().apply(followerConfig);
    }

    // ---- Shot consistency meter ----
    // Each ball pulls the wheel speed down (a "dip"). The speed just BEFORE the dip is the speed
    // that ball launched at. If balls come faster than the wheel recovers, later balls launch slow
    // and fall short. Dashboard: "Shots Counted", "Shots At Speed", "Last Shot Launch RPM",
    // "Last Shot Dip (RPM)", "Last Shot Recovery (s)". Button "Reset Shot Stats".
    private static final double kDipStart = 60;    // RPM below target that counts as a ball (Oct 27: 120 missed balls at lower RPM)
    private static final double kRecovered = 50;   // RPM below target that counts as recovered
    private static final double kAtSpeed = 100;    // launch within this of target = a good shot
    private double prevRpm = 0, minRpm = 0, launchRpm = 0, dipStart = 0;
    private boolean inDip = false;
    private int shots = 0, goodShots = 0;
    // Finished shots (dip seen AND recovered or gave up), for the shot log.
    private int finishedShots = 0;
    private double lastLaunch = 0, lastDip = 0, lastRecovery = 0, lastTarget = 0;
    public int finishedShots() { return finishedShots; }
    /** Balls that have left (counted at the start of each flywheel dip). */
    public int shotsStarted() { return shots; }
    /** FPGA time of the last ball leaving. */
    public double lastShotTime() { return lastShotTime; }
    private double lastShotTime = Double.NEGATIVE_INFINITY;
    public double lastLaunchRpm() { return lastLaunch; }
    public double lastDipRpm() { return lastDip; }
    public double lastRecoverySec() { return lastRecovery; }
    public double lastShotTargetRpm() { return lastTarget; }

    private void meterShots(double rpm) {
        double target = m_targetRpm;
        if (target < 1000) { inDip = false; prevRpm = rpm; return; }
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        if (!inDip && rpm < target - kDipStart && prevRpm >= target - kDipStart) {
            inDip = true; dipStart = now; minRpm = rpm; launchRpm = prevRpm; shots++; lastShotTime = now;
            if (Math.abs(launchRpm - target) <= kAtSpeed) goodShots++;
            SmartDashboard.putNumber("Last Shot Launch RPM", launchRpm);
            dev.doglog.DogLog.log("Shooter/ShotLaunchRpm", launchRpm);
            dev.doglog.DogLog.log("Shooter/ShotTargetRpm", target);
        } else if (inDip) {
            minRpm = Math.min(minRpm, rpm);
            if (rpm >= target - kRecovered) {
                inDip = false;
                lastLaunch = launchRpm; lastDip = target - minRpm; lastRecovery = now - dipStart; lastTarget = target;
                finishedShots++;
                SmartDashboard.putNumber("Last Shot Dip (RPM)", target - minRpm);
                SmartDashboard.putNumber("Last Shot Recovery (s)", now - dipStart);
                dev.doglog.DogLog.log("Shooter/ShotDipRpm", target - minRpm);
                dev.doglog.DogLog.log("Shooter/ShotRecoverySec", now - dipStart);
            } else if (now - dipStart > 1.0) {
                inDip = false; // never recovered (shooter can't reach target): stop waiting
                lastLaunch = launchRpm; lastDip = target - minRpm; lastRecovery = -1; lastTarget = target;
                finishedShots++;
            }
        }
        prevRpm = rpm;
        SmartDashboard.putNumber("Shots Counted", shots);
        SmartDashboard.putString("Shots At Speed", shots == 0 ? "-" : goodShots + " of " + shots
            + " (" + Math.round(100.0 * goodShots / shots) + "%)");
    }

    /** "Reset Shot Stats" dashboard button. */
    public edu.wpi.first.wpilibj2.command.Command resetShotStatsCommand() {
        return edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> { shots = 0; goodShots = 0; })
            .ignoringDisable(true).withName("Reset Shot Stats");
    }

    public double getCurrentRpm() {
        // Phoenix 6 returns rotations per second, so multiply by 60 to get RPM
        return leader.getVelocity().getValueAsDouble() * 60.0;
    }

    private static final String kBoostKey = "Shooter Recovery Boost (V)";

    private static double boostVolts() {
        if (!edu.wpi.first.wpilibj.Preferences.containsKey(kBoostKey)) {
            edu.wpi.first.wpilibj.Preferences.initDouble(kBoostKey, Constants.Shooter.kBoostVolts);
            SmartDashboard.putNumber(kBoostKey, Constants.Shooter.kBoostVolts);
        }
        double v = MathUtil.clamp(SmartDashboard.getNumber(kBoostKey,
            edu.wpi.first.wpilibj.Preferences.getDouble(kBoostKey, Constants.Shooter.kBoostVolts)), 0.0, 8.0);
        if (v != edu.wpi.first.wpilibj.Preferences.getDouble(kBoostKey, Constants.Shooter.kBoostVolts)) {
            edu.wpi.first.wpilibj.Preferences.setDouble(kBoostKey, v);
        }
        return v;
    }

    public void runAtRPM(double targetRpm) {
        // SAFETY CLAMP: keep the requested RPM inside +/- kMaxRPM
        targetRpm = MathUtil.clamp(targetRpm, -Constants.Shooter.kMaxRPM, Constants.Shooter.kMaxRPM);

        m_targetRpm = targetRpm;

        if (m_rampedSetpointRpm > targetRpm + 50.0) {
            // 1. Smooth spin-DOWN
            m_rampedSetpointRpm -= Constants.Shooter.kDecelerateStep;
            if (m_rampedSetpointRpm < targetRpm) {
                m_rampedSetpointRpm = targetRpm;
            }

        } else if (Math.abs(targetRpm - Constants.Shooter.kIdleRPM) < 1.0 && m_rampedSetpointRpm < targetRpm) {
            // 2. Smooth IDLE spin-up
            m_rampedSetpointRpm += Constants.Shooter.kIdleAccelerateStep;
            if (m_rampedSetpointRpm > targetRpm) {
                m_rampedSetpointRpm = targetRpm;
            }

        } else {
            // 3. Actual shooting: instant application for max acceleration
            m_rampedSetpointRpm = targetRpm;
        }

        // RECOVERY BOOST: well under target (a ball just left, or spinning up) = push extra volts
        // so the wheel is back at speed before the next ball. Off once within kBoostBelowRpm.
        double boost = 0.0;
        double measured = getCurrentRpm();
        if (m_rampedSetpointRpm > 500.0 && measured < m_rampedSetpointRpm - Constants.Shooter.kBoostBelowRpm) {
            boost = boostVolts();
        }
        SmartDashboard.putBoolean("Shooter Boosting", boost > 0);
        // Send the managed software setpoint to the TalonFX
        leader.setControl(m_velocityRequest.withVelocity(m_rampedSetpointRpm / 60.0).withFeedForward(boost));
        follower.setControl(m_velocityRequest.withVelocity(m_rampedSetpointRpm / 60.0).withFeedForward(boost));
    }

    /** Bench test: run only the leader motor. */
    public void testLeaderOnly(double rpm) {
        rpm = MathUtil.clamp(rpm, -Constants.Shooter.kMaxRPM, Constants.Shooter.kMaxRPM);
        m_targetRpm = rpm;
        m_rampedSetpointRpm = getCurrentRpm(); // sync the ramp tracker to reality
        leader.setControl(m_velocityRequest.withVelocity(rpm / 60.0));
        follower.stopMotor();
    }

    /** Bench test: run only the follower motor. */
    public void testFollowerOnly(double rpm) {
        rpm = MathUtil.clamp(rpm, -Constants.Shooter.kMaxRPM, Constants.Shooter.kMaxRPM);
        m_targetRpm = rpm;
        m_rampedSetpointRpm = getCurrentRpm(); // sync the ramp tracker to reality
        follower.setControl(m_velocityRequest.withVelocity(rpm / 60.0));
        leader.stopMotor();
    }

    public void stop() {
        m_targetRpm = 0.0;

        // Sync the ramp tracker to the physical wheel speed instead of dropping to 0,
        // so the handoff to the idle command stays butter-smooth after a shot.
        m_rampedSetpointRpm = getCurrentRpm();

        leader.stopMotor();
        follower.stopMotor();
    }

    @Override
    public void periodic() {
        // Live stator current (torque) so you can physically see the acceleration effort
        double currentDraw = Math.round(leader.getStatorCurrent().getValueAsDouble() * 10.0) / 10.0;

        SmartDashboard.putNumber("Shooter Actual RPM", getCurrentRpm());
        meterShots(getCurrentRpm());
        SmartDashboard.putNumber("Shooter Target RPM", m_targetRpm);
        SmartDashboard.putNumber("Shooter RPM Error", m_targetRpm == 0 ? 0 : getCurrentRpm() - m_targetRpm);
        SmartDashboard.putNumber("Shooter Torque Amps", currentDraw);
    }
}
