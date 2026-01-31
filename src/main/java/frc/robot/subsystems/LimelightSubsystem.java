package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightSubsystem extends SubsystemBase {
    private final NetworkTable table;
    private final NetworkTableEntry tx;   // Horizontal offset (degrees)
    private final NetworkTableEntry ty;   // Vertical offset (degrees)
    private final NetworkTableEntry ta;   // Target area (percent)
    private final NetworkTableEntry tv;   // Target valid (0 or 1)
    private final NetworkTableEntry botpose; // Array of 6 numbers: [x, y, z, roll, pitch, yaw]
    private final NetworkTableEntry botposeTargetSpace;

    public LimelightSubsystem() {
        // Adjust the table name if your limelight publishes under a different key.
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
        botpose = table.getEntry("botpose");
        botposeTargetSpace = table.getEntry("botpose_targetspace");
    }

    /** Returns the horizontal offset (tx) from the crosshair to the target in degrees. */
    public double getTX() {
        return tx.getNumber(0).doubleValue();
    }

    /** Returns the vertical offset (ty) from the crosshair to the target in degrees. */
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
        double x = poseArray[0];              // X position in meters
        double y = poseArray[1];              // Y position in meters
        double yawDegrees = poseArray[5];       // Yaw (rotation) in degrees
        return new Pose2d(x, y, Rotation2d.fromDegrees(yawDegrees));
    }

    public double[] getBotPoseTargetSpace() {
        return botposeTargetSpace.getDoubleArray(new double[6]);
    }

    @Override
    public void periodic() {
        // Publish vision data to SmartDashboard for tuning and debugging.
        SmartDashboard.putNumber("Limelight tx", getTX());
        SmartDashboard.putNumber("Limelight ty", getTY());
        SmartDashboard.putNumber("Limelight ta", getTargetArea());
        SmartDashboard.putNumber("Limelight tv", tv.getNumber(0).doubleValue());
    }
}
