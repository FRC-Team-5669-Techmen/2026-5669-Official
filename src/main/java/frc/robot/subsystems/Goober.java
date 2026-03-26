package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Goober extends SubsystemBase {
    private final TalonFX motor = new TalonFX(Constants.Turret.kMotorId);
    private final DutyCycleOut request = new DutyCycleOut(0);
    private final TalonFXConfiguration configs = new TalonFXConfiguration();

    private final PIDController turnPID = new PIDController(
        Constants.Turret.kP, Constants.Turret.kI, Constants.Turret.kD
    );

    // JITTER FIX: A moving average filter to smooth out Limelight pixel noise
    private final LinearFilter txFilter = LinearFilter.movingAverage(5);

    public Goober() {
        configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -25.0;
        configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 10.0; 
        configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        motor.getConfigurator().apply(configs);
        turnPID.setTolerance(Constants.Turret.kToleranceDegrees); 
    }

    public void setMotorSpeed(double percent) {
        motor.setControl(request.withOutput(percent * Constants.Turret.kSpeedMultiplier));
    }

    public void aimAtTarget(double rawTx) {
        // Run the raw Limelight data through our filter to smooth the noise
        double smoothedTx = txFilter.calculate(rawTx);

        double pidOutput = turnPID.calculate(smoothedTx, 0.0);
        
        // JITTER FIX: If we are close enough to the target, cut power completely
        // so the motor stops fighting tiny gearbox vibrations.
        if (turnPID.atSetpoint()) {
            stop();
        } else {
            double clampedOutput = MathUtil.clamp(
                pidOutput, 
                -Constants.Turret.kMaxOutput, 
                Constants.Turret.kMaxOutput
            );
            setMotorSpeed(-clampedOutput);
        }
    }

    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    public void stop() {
        motor.stopMotor();
    }

    public boolean targetLocked() {
        return turnPID.atSetpoint();
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Turret Motor Position", getPosition());

        //double currentPos = motor.getPosition().getValueAsDouble();
        //System.out.println("GOOBA:" + currentPos);
    }
}