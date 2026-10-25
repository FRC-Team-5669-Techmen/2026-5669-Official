package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Optional;
import java.util.stream.IntStream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

/**
 * Limelight AprilTag camera.
 *
 * <p>Offseason 2026 remount: the camera is mounted HORIZONTALLY and CENTERED on
 * the shooter, so the Limelight axes have their standard meanings:
 * <ul>
 *   <li>{@code tx} = horizontal angle to the tag (drives the turret aim)</li>
 *   <li>{@code ty} = vertical angle to the tag (drives the distance calculation)</li>
 * </ul>
 * No off-center aim correction is needed anymore.
 */
public class LimelightSubsystem extends SubsystemBase {
    private final NetworkTable table;
    private final NetworkTableEntry tx;   // Horizontal offset (degrees)
    private final NetworkTableEntry ty;   // Vertical offset (degrees)
    private final NetworkTableEntry ta;   // Target area (percent)
    private final NetworkTableEntry tv;   // Target valid (0 or 1)
    private final NetworkTableEntry botpose; // Array of 6 numbers: [x, y, z, roll, pitch, yaw]
    private final NetworkTableEntry botposeTargetSpace;
    private final NetworkTableEntry tid;

    // Tag IDs the turret may aim at right now: our hub's 3 once the alliance is known, both hubs before.
    private static final int[] kBothHubTagIds = IntStream.concat(
        Arrays.stream(Constants.Limelight.kRedHubTagIds),
        Arrays.stream(Constants.Limelight.kBlueHubTagIds)).toArray();
    private int[] allowedTagIds = kBothHubTagIds;
    private Optional<Alliance> filterAlliance = Optional.empty(); // alliance the filter was last set for

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

        // Tell the Limelight to ONLY track hub tags. periodic() narrows this to OUR hub
        // as soon as the Driver Station reports the alliance.
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight", allowedTagIds);
    }

    /** Horizontal angle to the target in degrees (positive = target to the right). */
    public double getTX() {
        return tx.getNumber(0).doubleValue();
    }

    /** Vertical angle to the target in degrees (positive = target above crosshair). */
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

    /** Returns true ONLY if the limelight sees a target AND it's one of OUR hub's tags. */
    public boolean isValidTarget() {
        if (!isTargetAvailable()) {
            return false;
        }

        int currentID = getID();
        for (int validID : allowedTagIds) {
            if (currentID == validID) {
                return true;
            }
        }

        return false;
    }

    /**
     * Distance to the target (meters) via trigonometry: the tag height is known,
     * so the vertical angle (ty) plus the camera mount angle gives range.
     * Returns 0.0 when no target is visible or the geometry has no solution.
     */
    public double distanceToTarget() {
        if (!isTargetAvailable()) return 0.0;

        double angleToGoalDeg = Constants.Limelight.kMountAngleDegrees + getTY();

        // Guard: at or below horizontal there is no valid solution. Without this,
        // a mismeasured mount angle would return infinite/negative distances and
        // poison the auto-aim filter. 0.0 is the same sentinel as "no target".
        if (angleToGoalDeg < 1.0) return 0.0;

        double angleToGoalRad = Math.toRadians(angleToGoalDeg);

        return (Constants.Limelight.kAprilTagHeightMeters - Constants.Limelight.kLensHeightMeters)
            / Math.tan(angleToGoalRad);
    }

    @Override
    public void periodic() {
        // Aim only at OUR hub once the Driver Station says which alliance we are. Keeps
        // checking, because the alliance can change before a match starts.
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (!alliance.equals(filterAlliance)) {
            filterAlliance = alliance;
            if (alliance.isEmpty()) {
                allowedTagIds = kBothHubTagIds;
            } else if (alliance.get() == Alliance.Red) {
                allowedTagIds = Constants.Limelight.kRedHubTagIds;
            } else {
                allowedTagIds = Constants.Limelight.kBlueHubTagIds;
            }
            LimelightHelpers.SetFiducialIDFiltersOverride("limelight", allowedTagIds);
        }

        // Publish vision data to SmartDashboard for tuning and debugging.
        SmartDashboard.putNumber("Limelight tx", getTX());
        SmartDashboard.putNumber("Limelight ty", getTY());
        SmartDashboard.putNumber("Limelight Tag ID", getID());
        SmartDashboard.putString("Limelight Aiming At", alliance.isEmpty() ? "both hubs (no alliance yet)"
            : alliance.get() == Alliance.Red ? "RED hub (tags 2, 5, 10)" : "BLUE hub (tags 18, 21, 26)");
        SmartDashboard.putNumber("Limelight distance (m)", distanceToTarget());
    }
}
