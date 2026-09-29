package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Goober;
import frc.robot.subsystems.MariosEar;

/**
 * Multi-camera turret aiming. Priority order:
 * <ol>
 *   <li>Limelight sees the tag → PID track it</li>
 *   <li>Left USB camera sees it → coarse swing left</li>
 *   <li>Right USB camera sees it → coarse swing right</li>
 *   <li>Nothing sees it → hold still</li>
 * </ol>
 */
public class MariosEarCommand extends Command {
    private final MariosEar vision;
    private final Goober turret;

    public MariosEarCommand(MariosEar vision, Goober turret) {
        this.vision = vision;
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        if (vision.limelightHasTarget()) {
            double tx = vision.getLimelightTX();
            turret.aimAtTarget(tx);
        } else if (vision.getLeftResult().hasTargets()) {
            turret.setMotorSpeed(-1.0); // Swing left to find tag
        } else if (vision.getRightResult().hasTargets()) {
            turret.setMotorSpeed(1.0);  // Swing right to find tag
        } else {
            turret.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
