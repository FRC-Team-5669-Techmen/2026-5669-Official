package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

/**
 * Limelight AprilTag camera (team nickname: "rizz").
 *
 * <p><b>IMPORTANT — the camera is mounted SIDEWAYS.</b> That swaps the meaning of
 * the Limelight's reported axes for this robot:
 * <ul>
 *   <li>{@code tx} = the real-world VERTICAL angle to the tag (used for distance)</li>
 *   <li>{@code ty} = the real-world HORIZONTAL angle to the tag (used for turret aim)</li>
 * </ul>
 * Getters below return the raw Limelight values; callers pick the right one for
 * their geometry (see {@link #distanceToTarget()} and {@link #getCorrectedTX()}).
 */
public class LimelightSubsystem extends SubsystemBase {
    private final NetworkTable table;
    private final NetworkTableEntry tx;   // Horizontal offset in camera frame (degrees)
    private final NetworkTableEntry ty;   // Vertical offset in camera frame (degrees)
    private final NetworkTableEntry ta;   // Target area (percent)
    private final NetworkTableEntry tv;   // Target valid (0 or 1)
    private final NetworkTableEntry botpose; // Array of 6 numbers: [x, y, z, roll, pitch, yaw]
    private final NetworkTableEntry botposeTargetSpace;
    private final NetworkTableEntry tid;

    public LimelightSubsystem() {
        // Adjust the table name if your limelight publishes under a different key.
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
        botpose = table.getEntry("botpose");
        botposeTargetSpace = table.getEntry("botpose_targetspace");
        tid = table.getEntry("tid");

        // Tell the Limelight to ONLY track the IDs in the Constants file
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight", Constants.Limelight.kValidTargetIds);
    }

    /** Raw tx (degrees). Because of the sideways mount, this is the VERTICAL angle to the tag. */
    public double getTX() {
        return tx.getNumber(0).doubleValue();
    }

    /** Raw ty (degrees). Because of the sideways mount, this is the HORIZONTAL angle to the tag. */
    public double getTY() {
        return ty.getNumber(0).doubleValue();
    }

    /** Returns the target area (ta) as a percentage of the image. */
    public double getTargetArea() {
        return ta.getNumber(0).doubleValue();
    }

    /** Returns true if the limelight sees a target (tv == 1). */
    public boolean isTargetAvailable() {
        return tv.getNumber(0).intValue() == 1;
    }

    /**
     * Returns the robot's pose as estimated by the Limelight's AprilTag detection.
     * Expects a "botpose" array of 6 numbers: [x, y, z, roll, pitch, yaw],
     * where x and y are in meters and yaw is in degrees.
     */
    public Pose2d getPose() {
        double[] poseArray = botpose.getDoubleArray(new double[6]);
        if (poseArray.length < 6) {
            // If no valid pose is available, return a default Pose2d.
            return new Pose2d();
        }
        double x = poseArray[0];          // X position in meters
        double y = poseArray[1];          // Y position in meters
        double yawDegrees = poseArray[5]; // Yaw (rotation) in degrees
        return new Pose2d(x, y, Rotation2d.fromDegrees(yawDegrees));
    }

    public double[] getBotPoseTargetSpace() {
        return botposeTargetSpace.getDoubleArray(new double[6]);
    }

    public int getID() {
        return (int) tid.getInteger(-1);
    }

    /** Returns true ONLY if the limelight sees a target AND its ID is in the approved list. */
    public boolean isValidTarget() {
        if (!isTargetAvailable()) {
            return false;
        }

        int currentID = getID();
        for (int validID : Constants.Limelight.kValidTargetIds) {
            if (currentID == validID) {
                return true;
            }
        }

        return false;
    }

    /**
     * Distance to the target (meters) via trigonometry: the tag height is known,
     * so the vertical angle (raw tx thanks to the sideways mount) gives range.
     */
    public double distanceToTarget() {
        if (!isTargetAvailable()) return 0.0;

        double angleToGoalDeg = Constants.Limelight.kMountAngleDegrees + getTX();
        double angleToGoalRad = Math.toRadians(angleToGoalDeg);

        return (Constants.Limelight.kAprilTagHeightMeters - Constants.Limelight.kLensHeightMeters)
            / Math.tan(angleToGoalRad);
    }

    /**
     * Horizontal aiming angle (degrees) for the turret, compensated for the camera
     * sitting {@link Constants.Limelight#kHOffsetMeters} off the shooter centerline.
     * Reads raw ty because the sideways mount makes ty the horizontal axis.
     */
    public double getCorrectedTX() {
        double horizontalAngle = getTY();
        double distance = distanceToTarget();

        if (distance < 0.5) {
            return horizontalAngle;
        }

        // Angular correction for the camera-to-shooter offset at this distance
        double correction = Math.toDegrees(Math.atan(Constants.Limelight.kHOffsetMeters / distance));

        // If camera is to the RIGHT, the shooter needs to aim further RIGHT (add)
        // If camera is to the LEFT, the shooter needs to aim further LEFT (subtract)
        return horizontalAngle + correction;
    }

    @Override
    public void periodic() {
        // Publish vision data to SmartDashboard for tuning and debugging.
        SmartDashboard.putNumber("Limelight tx", getTX());
    }
}
