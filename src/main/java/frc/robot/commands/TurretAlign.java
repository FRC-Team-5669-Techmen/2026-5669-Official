package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;

public class TurretAlign extends Command {
    private final LimelightSubsystem limelight;
    private final TurretSubsystem turret;

    // Start with a lower P for a Kraken (try 0.02 to 0.04)
    private final PIDController turnPID = new PIDController(0.03, 0.0, 0.002);

    public TurretAlign(LimelightSubsystem limelight, TurretSubsystem turret) {
        this.limelight = limelight;
        this.turret = turret;
        addRequirements(turret, limelight);
        turnPID.setTolerance(0.5); 
    }

    @Override
    public void execute() {
        if (!limelight.isTargetAvailable()) {
            turret.stop();
            return;
        }

        double tx = limelight.getTX();
        double pidOutput = turnPID.calculate(tx, 0.0);

        // CLAMP: Limits motor to 30% power so it doesn't overshoot wildly
        double clampedOutput = MathUtil.clamp(pidOutput, -0.3, 0.3);

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