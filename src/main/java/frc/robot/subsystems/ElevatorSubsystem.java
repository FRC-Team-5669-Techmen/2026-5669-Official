package frc.robot.subsystems;

import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;

/**
 * Climber: an elevator that rises, latches on, then pulls the robot up (Talon FX 19).
 * Hold Driver D-pad Up/Down to move it, release to stop. Positive = UP.
 *
 * <p>Soft limits: once kTopLimit and kBottomLimit are measured, the motor stops by itself
 * just before the top and the bottom, even if the D-pad is still held. No stalling into
 * the ends.
 */
public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(Elevator.kMotorId);

    // FOC off: this motor has no Phoenix Pro license, so it would refuse FOC commands.
    private final VoltageOut request = new VoltageOut(0).withEnableFOC(false);

    private boolean limitsOverridden = false; // true while the dashboard recovery button is on

    public ElevatorSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake; // holds still when released
        config.MotorOutput.Inverted = Elevator.kElevatorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.StatorCurrentLimit = Elevator.kStatorCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.SoftwareLimitSwitch = limitConfigs(limitsMeasured());
        motor.getConfigurator().apply(config);

        // NO zeroing here on purpose: deploying code must not re-zero wherever the elevator
        // happens to be, or the limits would end up in the wrong place. 0 comes from powering
        // on with the elevator fully DOWN (or the "Zero Elevator" dashboard button).
    }

    /** The limits are usable only once they're measured: top clearly above bottom. */
    private static boolean limitsMeasured() {
        return Elevator.kTopLimit > Elevator.kBottomLimit;
    }

    /** Soft limit settings. Forward = up (stops at the top), reverse = down (stops at the bottom). */
    private static SoftwareLimitSwitchConfigs limitConfigs(boolean on) {
        SoftwareLimitSwitchConfigs limits = new SoftwareLimitSwitchConfigs();
        limits.ForwardSoftLimitThreshold = Elevator.kTopLimit;
        limits.ForwardSoftLimitEnable = on;
        limits.ReverseSoftLimitThreshold = Elevator.kBottomLimit;
        limits.ReverseSoftLimitEnable = on;
        return limits;
    }

    private void setLimits(boolean on) {
        limitsOverridden = !on;
        motor.getConfigurator().apply(limitConfigs(on && limitsMeasured()));
    }

    /** Positive volts = up, negative = down. */
    public void setVolts(double volts) {
        motor.setControl(request.withOutput(volts));
    }

    public void stop() {
        motor.stopMotor();
    }

    /** Driver D-pad Up: hold to climb up, release to stop. Stops by itself at the top limit. */
    public Command upCommand() {
        return startEnd(() -> setVolts(Elevator.kElevatorVolts), this::stop);
    }

    /** Driver D-pad Down: hold to climb down, release to stop. Stops by itself at the bottom limit. */
    public Command downCommand() {
        return startEnd(() -> setVolts(-Elevator.kElevatorVolts), this::stop);
    }

    /** The "Zero Elevator (elevator is DOWN)" dashboard button: call this spot 0. Works while disabled. */
    public Command zeroCommand() {
        return runOnce(() -> motor.setPosition(0)).ignoringDisable(true).withName("Zero Elevator");
    }

    /**
     * The "Elevator Limits OFF (recovery)" dashboard button: click to turn the limits off,
     * click again to turn them back on. Only for when the position is wrong (powered on
     * with the elevator up) and a limit blocks the D-pad. Nothing stops it at the ends
     * while it's on, so go slowly, bring it all the way down, press Zero Elevator, then
     * click this again.
     */
    public Command limitsOffCommand() {
        // No subsystem requirement on purpose, so using the D-pad doesn't turn the limits back on.
        return Commands.startEnd(() -> setLimits(false), () -> setLimits(true))
            .ignoringDisable(true).withName("Elevator Limits OFF");
    }

    private String status() {
        if (!limitsMeasured()) {
            return "No limits yet: measure top/bottom";
        }
        if (limitsOverridden) {
            return "LIMITS OFF (recovery)";
        }
        double position = motor.getPosition().getValueAsDouble();
        if (position >= Elevator.kTopLimit) {
            return "At TOP limit";
        }
        if (position <= Elevator.kBottomLimit) {
            return "At BOTTOM limit";
        }
        return "OK";
    }

    // CLIMB WHILE SHOOTING: other code checks this so nothing automatic moves the robot or the
    // intake while it is climbing (heading assist, agitation shimmy, intake pumps, vacuum deploy).
    private static volatile boolean raised = false;

    /** The elevator is up off the bottom (climbing, or about to). */
    public static boolean isRaised() { return raised; }

    @Override
    public void periodic() {
        raised = motor.getPosition().getValueAsDouble() > Elevator.kRaisedThreshold;
        SmartDashboard.putBoolean("Climbing (elevator up)", raised);
        SmartDashboard.putString("Elevator Status", status());
        SmartDashboard.putNumber("Elevator Position (rot)", motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Amps", motor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Elevator Speed (rot/s)", motor.getVelocity().getValueAsDouble());
    }
}
