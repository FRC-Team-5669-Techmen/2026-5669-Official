package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeRoller;

/**
 * Ground intake ROLLER (Talon FX 22): the spinning wheels that pick up balls. Hold Driver
 * Right Trigger to spin it in, hold Driver Left Trigger to spit balls back out.
 *
 * <p>While kIntakeRollerEnabled is false, the motor is never created (so there are no
 * CAN errors) and the triggers do nothing.
 */
public class IntakeRollerSubsystem extends SubsystemBase {
    // Stays null while the roller is disabled. Every method checks for that.
    private final TalonFX motor;

    // FOC off: this motor has no Phoenix Pro license.
    private final VoltageOut request = new VoltageOut(0).withEnableFOC(false);

    public IntakeRollerSubsystem() {
        if (IntakeRoller.kIntakeRollerEnabled) {
            motor = new TalonFX(IntakeRoller.kMotorId);
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // rollers spin down freely
            config.MotorOutput.Inverted = IntakeRoller.kInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
            config.CurrentLimits.StatorCurrentLimit = IntakeRoller.kStatorCurrentLimit;
            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = IntakeRoller.kSupplyCurrentLimit;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            motor.getConfigurator().apply(config);
        } else {
            motor = null;
        }
    }

    // Oct 28 (robot): with Talon 22 off the bus, reading or commanding it every loop printed CAN
    // errors to the Driver Station and overran the loop. Checked every 2 s; skipped while missing.
    private boolean onlineCache = true;
    private double onlineCheckedAt = -10;

    private boolean online() {
        double t = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        if (t - onlineCheckedAt > 2.0) {
            onlineCheckedAt = t;
            onlineCache = motor.isConnected(0.5);
        }
        return onlineCache;
    }

    public void run() {
        setVolts(IntakeRoller.kRollerVolts);
    }

    /** Positive = balls in, negative = balls out. */
    public void setVolts(double volts) {
        if (motor != null && online()) {
            motor.setControl(request.withOutput(volts));
        }
    }

    public void stop() {
        if (motor != null && online()) {
            motor.stopMotor();
        }
    }

    /** Driver Right Trigger: hold to spin, release to stop. Does nothing while disabled. */
    public Command runCommand() {
        return startEnd(this::run, this::stop);
    }

    /** Driver Left Trigger: hold to spin backwards (balls out), release to stop. */
    public Command reverseCommand() {
        return startEnd(() -> setVolts(IntakeRoller.kReverseVolts), this::stop);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake Roller Enabled", motor != null);
        SmartDashboard.putBoolean("Intake Roller On CAN", motor != null && online());
        if (motor != null && online()) {
            // While Right Trigger is held, speed should be well above 0.
            SmartDashboard.putNumber("Intake Roller Speed (rot/s)", motor.getVelocity().getValueAsDouble());
            SmartDashboard.putNumber("Intake Roller Amps", motor.getStatorCurrent().getValueAsDouble());
        }
    }
}
