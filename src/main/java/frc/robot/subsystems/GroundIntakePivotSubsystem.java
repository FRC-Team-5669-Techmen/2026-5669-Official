package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Raises and lowers the ground intake with two motors working as a pair.
 *
 * <p>Only the leader (motor 20) is ever given commands. The follower (motor 21) is
 * told once, at startup, to copy the leader, and keeps doing that from then on.
 * Never send the follower any other command: it would stop following, and the two
 * motors could fight each other.
 *
 * <p>No position control or soft limits yet. Nothing stops the intake at the end of
 * its travel except the driver letting go of the button.
 */
public class GroundIntakePivotSubsystem extends SubsystemBase {
    private final TalonFX leader = new TalonFX(Constants.GroundIntakePivot.kLeaderId);
    private final TalonFX follower = new TalonFX(Constants.GroundIntakePivot.kFollowerId);

    // FOC must be OFF. CTRE turns on "FOC" by default, but FOC is a paid Phoenix Pro
    // feature. An unlicensed motor that gets an FOC command refuses to drive (its LED
    // alternates red/green). That's why motor 20 sat still while 21 copied it.
    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(false);

    public GroundIntakePivotSubsystem() {
        // Settings for BOTH motors: brake when stopped, the stator current limit,
        // and the direction setting. Giving both motors identical settings means
        // flipping kLeaderInverted can never leave the pair disagreeing.
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = Constants.GroundIntakePivot.kLeaderInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.StatorCurrentLimit = Constants.GroundIntakePivot.kStatorCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        leader.getConfigurator().apply(config);
        follower.getConfigurator().apply(config);

        // Tell motor 21 to copy motor 20. Opposed = spin the opposite way (mirrored mount).
        follower.setControl(new Follower(
            Constants.GroundIntakePivot.kLeaderId,
            Constants.GroundIntakePivot.kFollowerAlignment));
    }

    /** Positive volts = raise, negative volts = lower. Only the leader is commanded. */
    public void setVolts(double volts) {
        leader.setControl(voltageRequest.withOutput(volts));
    }

    /** Stops both motors. The follower copies the leader, so it stops too. */
    public void stop() {
        leader.stopMotor();
    }

    /** Hold the button to raise, let go to stop. */
    public Command raiseCommand() {
        return startEnd(() -> setVolts(Constants.GroundIntakePivot.kMoveVolts), this::stop);
    }

    /** Hold the button to lower, let go to stop. */
    public Command lowerCommand() {
        return startEnd(() -> setVolts(-Constants.GroundIntakePivot.kMoveVolts), this::stop);
    }

    @Override
    public void periodic() {
        // Watch these while testing. If both currents climb high but the intake doesn't
        // move, the motors are fighting: flip kFollowerAlignment.
        SmartDashboard.putNumber("Intake Pivot Leader Amps", leader.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake Pivot Follower Amps", follower.getStatorCurrent().getValueAsDouble());
        // Is each motor actually turning? While a D-pad button is held, both should read
        // about the same size number with OPPOSITE signs (they're mirrored). 0 = not moving.
        SmartDashboard.putNumber("Intake Pivot Leader Speed (rot/s)", leader.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake Pivot Follower Speed (rot/s)", follower.getVelocity().getValueAsDouble());
        // Write down the readings at fully raised and fully lowered, for soft limits later.
        SmartDashboard.putNumber("Intake Pivot Position (rot)", leader.getPosition().getValueAsDouble());
    }
}
