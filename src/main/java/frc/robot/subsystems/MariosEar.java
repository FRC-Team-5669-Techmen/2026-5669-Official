package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * RETIRED (Oct 2026): the "Left" and "Right" PhotonVision USB cameras aren't on the robot
 * (the Limelight is the only camera), so RobotContainer doesn't create this. Kept in case
 * USB cameras come back.
 */
public class MariosEar extends SubsystemBase {
    // Nicknames must match the PhotonVision Dashboard
    private final PhotonCamera camLeft = new PhotonCamera("Left");
    private final PhotonCamera camRight = new PhotonCamera("Right");
    private final LimelightSubsystem limelight;

    public MariosEar(LimelightSubsystem limelight) {
        this.limelight = limelight;
    }

    public boolean limelightHasTarget() {
        return limelight.isTargetAvailable();

    }

    public double getLimelightTX() {
        return limelight.getTX();
    }

     public PhotonPipelineResult getLeftResult() {
        return camLeft.getLatestResult();
    }

   public PhotonPipelineResult getRightResult() {
        return camRight.getLatestResult();
    }
}