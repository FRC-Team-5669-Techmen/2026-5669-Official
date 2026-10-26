package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.Constants.Aim;
import frc.robot.LimelightHelpers;
import frc.robot.aiming.ShotSolver;

/**
 * Keeps the drivetrain's field position right using the Limelight (shoot from anywhere).
 *
 * <p>The Limelight rides on the turret, so every loop this tells it where the camera actually is
 * on the robot (from the turret angle). Then it reads two estimates:
 * <ul>
 *   <li>MegaTag1: position AND heading from the tags alone. Only trusted with 2+ tags. Used to set
 *       the heading at the start and to slowly correct gyro drift.</li>
 *   <li>MegaTag2: position from the tags using our gyro heading. Works with a single tag, so it
 *       does most of the work while driving. Only used once the heading is known.</li>
 * </ul>
 * Fixes are thrown away while the turret or robot spins fast (camera angle unknown, blur) and
 * when a single tag is far away. Dashboard: "Vision Status", "Vision Tags", "Pose Trusted".
 */
public class VisionLocalizer extends SubsystemBase {
    private static final String kName = "limelight";

    private final CommandSwerveDrivetrain drivetrain;
    private final Goober turret;
    private final GoobaSubsystem hood;
    private double lastHoodPos = 0.0;
    private double hoodSpeed = 0.0; // rotations per second

    private boolean headingKnown = false;   // gyro heading matches the field (seeded)
    private boolean positionKnown = false;  // a vision fix arrived, or an auto set the pose
    private double lastAcceptedTime = -100; // FPGA seconds of the last accepted fix
    private String status = "starting";
    private int accepted = 0;

    public VisionLocalizer(CommandSwerveDrivetrain drivetrain, Goober turret, GoobaSubsystem hood) {
        this.drivetrain = drivetrain;
        this.turret = turret;
        this.hood = hood;
        SmartDashboard.putBoolean("Vision Pose ON", true);
    }

    /**
     * The gyro heading now matches the field: called when the driver resets the gyro facing away
     * from the driver station (Driver LB) and when an auto starts (the path sets the pose).
     */
    public void markHeadingKnown() {
        headingKnown = true;
    }

    /**
     * Dashboard "Re-find Position (camera)": forget the heading and position so the next steady
     * 2-tag view sets them again (robot still, both tags of a hub face in view). Works disabled.
     */
    public edu.wpi.first.wpilibj2.command.Command refindCommand() {
        return edu.wpi.first.wpilibj2.command.Commands.runOnce(() -> {
            headingKnown = false;
            positionKnown = false;
            seedCount = 0;
            lastSeedPose = null;
            System.out.println("LOCALIZATION | position forgotten: hold the robot still in view of 2 hub tags");
        }).ignoringDisable(true).withName("Re-find Position (camera)");
    }

    /** An auto is starting: its path sets the whole pose. */
    public void markPoseSetByAuto() {
        headingKnown = true;
        positionKnown = true;
    }

    /**
     * True once the heading is known AND the position has been set (a vision fix or an auto).
     * After that, wheel odometry carries the position between tag sightings; each new fix
     * corrects it. "Seconds Since Vision Fix" on the dashboard shows how fresh it is.
     */
    public boolean poseTrusted() {
        return headingKnown && positionKnown;
    }

    /** True once the heading is known (gyro reset or auto start or a 2-tag fix). */
    public boolean headingKnown() {
        return headingKnown;
    }

    @Override
    public void periodic() {
        boolean on = SmartDashboard.getBoolean("Vision Pose ON", true);

        // 1. Where the camera is on the robot right now (it turns with the turret).
        boolean turretKnown = turret.endMeasured();
        double turretDeg = turret.getAngleDegrees();
        double[] cam = ShotSolver.cameraInRobot(
            new Translation2d(Aim.kTurretPivotX, Aim.kTurretPivotY), Aim.kHomeYawDegrees, turretDeg,
            Aim.kCameraForwardOfPivot, Aim.kCameraLeftOfPivot);
        double side = Aim.kLimelightSideIsRight ? -cam[1] : cam[1];
        double yaw = Aim.kLimelightYawCCW ? cam[2] : -cam[2];
        // The camera rides on the hood: height and tilt follow the hood position.
        double hoodPos = hood.getPosition();
        hoodSpeed = (hoodPos - lastHoodPos) / 0.02;
        lastHoodPos = hoodPos;
        double f = Constants.Gooba.kHoodEnabled
            ? MathUtil.clamp((hoodPos - Constants.Gooba.kBottomPosition)
                / (Constants.Gooba.kTopPosition - Constants.Gooba.kBottomPosition), 0.0, 1.0)
            : 0.0;
        double height = MathUtil.interpolate(Aim.kCameraHeightHoodDown, Aim.kCameraHeightHoodUp, f);
        double pitch = MathUtil.interpolate(Aim.kCameraPitchHoodDown, Aim.kCameraPitchHoodUp, f);
        LimelightHelpers.setCameraPose_RobotSpace(kName, cam[0], side, height, 0.0, pitch, yaw);
        SmartDashboard.putNumber("Camera Height (in)", height / 0.0254);
        SmartDashboard.putNumber("Camera Pitch (deg)", pitch);

        // 2. MegaTag2 needs our heading in field (blue-origin) degrees.
        var state = drivetrain.getState();
        double headingDeg = state.Pose.getRotation().getDegrees();
        double spin = state.Speeds.omegaRadiansPerSecond;
        LimelightHelpers.SetRobotOrientation(kName, headingDeg, Math.toDegrees(spin), 0, 0, 0, 0);

        status = evaluate(on, turretKnown, spin, state.Speeds);
        SmartDashboard.putString("Vision Status", status);
        SmartDashboard.putBoolean("Pose Trusted", poseTrusted());
        SmartDashboard.putBoolean("Heading Known", headingKnown);
        SmartDashboard.putNumber("Vision Fixes Accepted", accepted);
        SmartDashboard.putNumber("Seconds Since Vision Fix", Timer.getFPGATimestamp() - lastAcceptedTime);
        SmartDashboard.putNumberArray("Camera In Robot (fwd,left,yaw)", cam);
    }

    private static final int kSeedFrames = 10;     // camera frames in a row that must agree
    private int seedCount = 0;
    private double lastSeedTime = -1;
    private edu.wpi.first.math.geometry.Pose2d lastSeedPose = null;

    /** Heading unknown, 2+ tags: set the whole pose from the camera once it's steady. */
    private String seedFromCamera(LimelightHelpers.PoseEstimate mt1, edu.wpi.first.math.kinematics.ChassisSpeeds speeds) {
        boolean still = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) < 0.2
            && Math.abs(speeds.omegaRadiansPerSecond) < 0.2;
        if (!still) { seedCount = 0; return "heading: hold the robot still (2 tags in view)"; }
        if (mt1.timestampSeconds == lastSeedTime) return "heading: steady " + seedCount + "/" + kSeedFrames;
        lastSeedTime = mt1.timestampSeconds;
        if (lastSeedPose != null
                && mt1.pose.getTranslation().getDistance(lastSeedPose.getTranslation()) < 0.15
                && Math.abs(mt1.pose.getRotation().minus(lastSeedPose.getRotation()).getDegrees()) < 3.0) {
            seedCount++;
        } else {
            seedCount = 1;
        }
        lastSeedPose = mt1.pose;
        if (seedCount < kSeedFrames) return "heading: steady " + seedCount + "/" + kSeedFrames;
        drivetrain.resetPose(mt1.pose);
        headingKnown = true;
        seedCount = 0;
        System.out.println(String.format("LOCALIZATION | pose SET by the camera: x %.2f y %.2f heading %.1f deg (%d tags). Driver LB overrides the heading.",
            mt1.pose.getX(), mt1.pose.getY(), mt1.pose.getRotation().getDegrees(), mt1.tagCount));
        return accept("SET from camera, " + mt1.tagCount + " tags");
    }

    private String evaluate(boolean on, boolean turretKnown, double spin, edu.wpi.first.math.kinematics.ChassisSpeeds speeds) {
        if (!on) return "OFF (dashboard switch)";
        if (!Aim.kGeometryMeasured) return "OFF: turret/camera geometry not measured (Constants.Aim)";
        if (!turretKnown) return "OFF: turret far end not set";
        if (Math.abs(turret.getVelocityDegPerSec()) > Aim.kMaxTurretSpeedForVision) return "skip: turret turning fast";
        if (Math.abs(spin) > Aim.kMaxRobotSpinForVision) return "skip: robot spinning fast";
        if (Math.abs(hoodSpeed) > Aim.kMaxHoodSpeedForVision) return "skip: hood moving";

        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(kName);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kName);
        int tags = mt1 == null ? 0 : mt1.tagCount;
        SmartDashboard.putNumber("Vision Tags", tags);
        if (tags == 0) return "no tags";

        // MegaTag1 with 2+ tags: position AND heading.
        // Heading UNKNOWN: wait for kSeedFrames camera frames in a row that agree (robot still),
        // then SET the pose to exactly that (resetPose). Blending a single fix only moved the
        // heading part of the way and then locked it (Oct 25: Aim Check read -53 and -96 deg).
        // Heading KNOWN: position only. The heading belongs to the gyro and Driver LB, because the
        // drive's field-centric "forward" uses it.
        if (mt1.tagCount >= 2 && mt1.avgTagDist < 6.0) {
            DogLog.log("Vision/MT1Pose", mt1.pose);
            if (!headingKnown) {
                return seedFromCamera(mt1, speeds);
            }
            double xy = xyStdDev(mt1.avgTagDist, mt1.tagCount);
            drivetrain.addVisionMeasurement(mt1.pose, mt1.timestampSeconds, VecBuilder.fill(xy, xy, 1e6));
            return accept("MT1 " + mt1.tagCount + " tags");
        }
        seedCount = 0;

        // MegaTag2 (single tag OK): needs the heading to be right first.
        if (!headingKnown) return "1 tag: waiting for heading (Driver LB facing away, or 2 tags)";
        if (mt2 == null || mt2.tagCount == 0) return "no MT2";
        if (mt2.tagCount == 1 && mt2.avgTagDist > Aim.kMaxSingleTagDistance) return "skip: 1 tag too far";
        double xy = xyStdDev(mt2.avgTagDist, mt2.tagCount);
        drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds, VecBuilder.fill(xy, xy, 1e6));
        DogLog.log("Vision/MT2Pose", mt2.pose);
        return accept("MT2 " + mt2.tagCount + " tag" + (mt2.tagCount > 1 ? "s" : ""));
    }

    private String accept(String what) {
        positionKnown = true;
        lastAcceptedTime = Timer.getFPGATimestamp();
        accepted++;
        return "OK " + what;
    }

    /** Position uncertainty grows with distance squared and shrinks with more tags. */
    private static double xyStdDev(double avgDist, int tags) {
        return Aim.kVisionXYStdDevAt1m * Math.max(1.0, avgDist * avgDist) / tags;
    }
}
