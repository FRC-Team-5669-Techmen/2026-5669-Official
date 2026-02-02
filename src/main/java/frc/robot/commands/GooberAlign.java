package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Goober;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;

public class GooberAlign extends Command {
    private final LimelightSubsystem limelight;
    private final Goober turret;

    private final PIDController turnPID = new PIDController(
        Constants.Turret.kP, 
        Constants.Turret.kI, 
        Constants.Turret.kD
    );

    public GooberAlign(LimelightSubsystem limelight, Goober turret) {
        this.limelight = limelight;
        this.turret = turret;
        addRequirements(turret, limelight);
        
        turnPID.setTolerance(Constants.Turret.kToleranceDegrees); 
    }

    @Override
    public void execute() {
        if (!limelight.isTargetAvailable()) {
            turret.stop();
            return;
        }

        double tx = limelight.getTX();
        double pidOutput = turnPID.calculate(tx, 0.0);

        double clampedOutput = MathUtil.clamp(
            pidOutput, 
            -Constants.Turret.kMaxOutput, 
            Constants.Turret.kMaxOutput
        );

        turret.setMotorSpeed(clampedOutput);
    }

    @Override
    public boolean isFinished() {
        return limelight.isTargetAvailable() && turnPID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }
}