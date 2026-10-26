package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterIntakeSubsystem extends SubsystemBase {
    private final TalonFX motor = new TalonFX(Constants.ShooterIntake.kMotorId);
    // FOC off: this motor has no Phoenix Pro license, and an unlicensed motor refuses FOC commands.
    private final DutyCycleOut request = new DutyCycleOut(0).withEnableFOC(false);
    // Forward feeding uses VOLTS, not percent: "100%" means whatever the battery has (12.8 V fresh,
    // ~11 V tired), so the ball's entry speed into the shooter changed with the battery. A fixed
    // voltage below the sagged battery gives the same entry speed every shot (consistent range).
    private final VoltageOut feedRequest = new VoltageOut(0).withEnableFOC(false);

    public ShooterIntakeSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = Constants.ShooterIntake.kInverted ? 
            com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive : 
            com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
            
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // REDLINE LIMITS: Maximum power without triggering hardware fault
        CurrentLimitsConfigs currentLimits = config.CurrentLimits;
        currentLimits.SupplyCurrentLimit = 60.0; // Max draw from battery
        currentLimits.SupplyCurrentLimitEnable = true;

        currentLimits.StatorCurrentLimit = 80.0; // Max push to motor coils
        currentLimits.StatorCurrentLimitEnable = true;

        motor.getConfigurator().apply(config);
        SmartDashboard.putNumber("Uptake Forward Volts", Constants.ShooterIntake.kForwardVolts);
    }

    /** Feed the shooter at the dashboard's "Uptake Forward Volts" (battery-proof). */
    public void feed() {
        double volts = MathUtil.clamp(SmartDashboard.getNumber("Uptake Forward Volts", Constants.ShooterIntake.kForwardVolts), 0.0, 12.0);
        motor.setControl(feedRequest.withOutput(volts));
    }

    public void run(double speed) {
        motor.setControl(request.withOutput(speed));
    }

    public void stop() {
        motor.stopMotor();
    }
}