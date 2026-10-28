package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(Constants.Index.kMotorId);
    
    // FOC off: this motor has no Phoenix Pro license, and an unlicensed motor refuses
    // FOC commands. (Only the shooter motors 16 and 17 are licensed.)
    private final VoltageOut request = new VoltageOut(0).withEnableFOC(false);

    public IndexSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = Constants.Index.kInverted ? 
            com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive : 
            com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
        
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // SAFETY GLOVES OFF: These limits match your main shooter flywheel
        CurrentLimitsConfigs currentLimits = config.CurrentLimits;
        
        // Let it pull up to 80 Amps from the battery instantly
        currentLimits.SupplyCurrentLimit = 80.0; 
        currentLimits.SupplyCurrentLimitEnable = true;

        // Let it push up to 160 Amps of torque to the physical coils
        currentLimits.StatorCurrentLimit = 80.0; // Oct 25: was 160; 80 A at 16:1 still pushes ~250 N at the omni rim, and spares the gearbox in a jam 
        currentLimits.StatorCurrentLimitEnable = true;
        
        // Soften forward/reverse flips (auto-unjam, shakes) on the MAXPlanetary.
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Agitation.kSpindexRampSeconds;
        motor.getConfigurator().apply(config);
        // The tuned value is saved on the roboRIO, so Shot Map Day tuning survives reboots.
        edu.wpi.first.wpilibj.Preferences.initDouble("Spindex Forward Volts", Constants.Index.kForwardSpeed);
        SmartDashboard.putNumber("Spindex Forward Volts", edu.wpi.first.wpilibj.Preferences.getDouble("Spindex Forward Volts", Constants.Index.kForwardSpeed));
    }

    /**
     * Forward feed volts: dashboard "Spindex Forward Volts" (0-12, default 12). Lets the team
     * trim the spindexer speed live after a gear change, without redeploying.
     */
    public static double forwardVolts() {
        return edu.wpi.first.math.MathUtil.clamp(SmartDashboard.getNumber("Spindex Forward Volts", Constants.Index.kForwardSpeed), 0.0, 12.0);
    }

    // Automatic unjam (see frc.robot.feeding.JamDetector and Constants.Agitation).
    private final frc.robot.feeding.JamDetector jam = new frc.robot.feeding.JamDetector(
        Constants.Agitation.kJamAmps, Constants.Agitation.kJamSpeed, Constants.Agitation.kJamSeconds,
        Constants.Agitation.kJamIgnoreAfterStart, Constants.Agitation.kUnjamSeconds,
        Constants.Agitation.kJamBurstWindow, Constants.Agitation.kJamBurstCount, Constants.Agitation.kUnjamLockout);
    private double lastFeedCall = Double.NEGATIVE_INFINITY;

    /**
     * SHOOTING feed: forward at "Spindex Forward Volts", with automatic unjam. If the spindexer
     * stalls (two balls fighting for the uptake) it reverses for a moment by itself, then feeds
     * again. Every shooting command calls this every loop.
     */
    public void feed() {
        feed(forwardVolts());
    }

    /** SHOOTING feed at a given forward voltage (passing: as fast as it goes), with automatic unjam. */
    public void feed(double forwardVolts) {
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        lastFeedCall = now;
        boolean reverse = jam.update(now, motor.getStatorCurrent().getValueAsDouble(),
            motor.getVelocity().getValueAsDouble());
        motor.setControl(request.withOutput(reverse ? Constants.Agitation.kUnjamVolts
            : edu.wpi.first.math.MathUtil.clamp(forwardVolts, 0.0, 12.0)));
    }

    /** True while a shooting command is feeding (including its short unjam/shake reversals). */
    public boolean isFeeding() {
        return edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - lastFeedCall < 0.1;
    }

    /** Agitator: reverse for a blink (shakes a bridged pile when the intake can't pump). */
    public void shake(double seconds) {
        jam.shake(edu.wpi.first.wpilibj.Timer.getFPGATimestamp(), seconds);
    }

    public int jamCount() { return jam.jams(); }
    public double tempCelsius() { return motor.getDeviceTemp().getValueAsDouble(); }

    /** Fixed volts (Operator B unjam, intake rewind). Not a shooting feed: no auto-unjam. */
    public void run(double volts) {
        jam.stopped();
        motor.setControl(request.withOutput(volts));
    }

    public void stop() {
        jam.stopped();
        motor.stopMotor();
    }

    /** "Spindex Speed (rot/s)" and "Spindex Amps": a slow speed with high amps = jammed or geared too low. */
    @Override
    public void periodic() {
        double v = SmartDashboard.getNumber("Spindex Forward Volts", Constants.Index.kForwardSpeed);
        if (v != edu.wpi.first.wpilibj.Preferences.getDouble("Spindex Forward Volts", Constants.Index.kForwardSpeed)) {
            edu.wpi.first.wpilibj.Preferences.setDouble("Spindex Forward Volts", v);
        }
        SmartDashboard.putNumber("Spindex Speed (rot/s)", motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Spindex Amps", motor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Spindex Volts", motor.getMotorVoltage().getValueAsDouble());
    }
}
