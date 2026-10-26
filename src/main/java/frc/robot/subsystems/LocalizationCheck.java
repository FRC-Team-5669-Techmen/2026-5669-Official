package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Aim;
import frc.robot.LimelightHelpers;
import frc.robot.aiming.ShotSolver;

/**
 * Beginner-friendly checks for shoot-from-anywhere. Nothing here changes how the robot drives or
 * shoots; it only watches and reports. Results go to the dashboard AND the Driver Station console
 * (lines starting with "LOCALIZATION"), which the Driver Station saves on the laptop.
 *
 * <ul>
 *   <li>"Localization Step": one plain-English line saying what to do next.</li>
 *   <li>"Aim Check (deg)": whenever one of OUR hub tags is in view, compares where the robot THINKS
 *       that tag is (from its field position) with where the camera actually SEES it. Near 0 =
 *       the position and the geometry are right. Works while disabled: just push the robot around.</li>
 *   <li>"Test: Turret Sweep" button (robot ENABLED, sitting still, tags in view): turns the turret
 *       slowly end to end and checks the vision position stays put. Tells you which setting to
 *       flip if not.</li>
 * </ul>
 */
public class LocalizationCheck extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;
    private final Goober turret;
    private final LimelightSubsystem limelight;
    private final VisionLocalizer localizer;
    private final AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    /** Aim Check passes within this many degrees (camera tilt makes tx a little imperfect). */
    static final double kAimCheckPassDeg = 3.0;
    private double aimCheckDeg = Double.NaN;
    private int aimCheckGood = 0, aimCheckBad = 0;

    public LocalizationCheck(CommandSwerveDrivetrain drivetrain, Goober turret,
                             LimelightSubsystem limelight, VisionLocalizer localizer) {
        this.drivetrain = drivetrain;
        this.turret = turret;
        this.limelight = limelight;
        this.localizer = localizer;
        SmartDashboard.putString("Turret Sweep Result", "not run yet");
    }

    // ---------------- what to do next ----------------

    private String nextStep() {
        if (!Aim.kGeometryMeasured) return "Turret/camera geometry not entered (Constants.Aim)";
        if (!turret.endMeasured()) return "Set the turret far end first (dashboard button)";
        if (!SmartDashboard.getBoolean("Vision Pose ON", true)) return "Vision is switched OFF (Vision Pose ON)";
        String mode = LimelightSubsystem.practiceFieldMode() ? "[PRACTICE FIELD: hub tags only] " : "";
        int tags = (int) SmartDashboard.getNumber("Vision Tags", 0);
        if (tags == 0) return mode + "1) Point the Limelight at AprilTags (1-4 m away)";
        if (!localizer.headingKnown()) return mode + "2) See 2+ tags at once, or press Driver LB facing away from your driver station";
        if (!localizer.poseTrusted()) return mode + "3) Waiting for the first good vision fix";
        if (Double.isNaN(aimCheckDeg)) return mode + "4) Position OK. Point at OUR hub's tags to run the Aim Check";
        if (Math.abs(aimCheckDeg) <= kAimCheckPassDeg) return mode + "5) Aim Check PASS. Run Test: Turret Sweep, then shoot";
        return mode + "Aim Check FAIL (" + Math.round(aimCheckDeg) + " deg): run Test: Turret Sweep";
    }

    // ---------------- Aim Check (passive) ----------------

    private void updateAimCheck() {
        aimCheckDeg = Double.NaN;
        if (!localizer.poseTrusted() || !turret.endMeasured()) return;
        LimelightHelpers.RawFiducial tag = limelight.getHubTag();
        if (tag == null) return;
        var tagPose = field.getTagPose(tag.id);
        if (tagPose.isEmpty()) return;

        Pose2d robot = drivetrain.getState().Pose;
        double[] cam = ShotSolver.cameraInRobot(new Translation2d(Aim.kTurretPivotX, Aim.kTurretPivotY),
            Aim.kHomeYawDegrees, turret.getAngleDegrees(), Aim.kCameraForwardOfPivot, Aim.kCameraLeftOfPivot);
        Translation2d camField = robot.getTranslation().plus(new Translation2d(cam[0], cam[1]).rotateBy(robot.getRotation()));
        double camYawField = robot.getRotation().getDegrees() + cam[2];
        Translation2d toTag = tagPose.get().toPose2d().getTranslation().minus(camField);
        double bearing = Math.toDegrees(Math.atan2(toTag.getY(), toTag.getX()));
        // Limelight tx is positive to the RIGHT (clockwise); field bearings are counter-clockwise.
        double predictedTx = -ShotSolver.wrap180(bearing - camYawField);
        aimCheckDeg = tag.txnc - predictedTx;
        if (Math.abs(aimCheckDeg) <= kAimCheckPassDeg) aimCheckGood++; else aimCheckBad++;
        DogLog.log("Localization/AimCheckDeg", aimCheckDeg);
        DogLog.log("Localization/AimCheckTag", tag.id);
    }

    @Override
    public void periodic() {
        updateAimCheck();
        String step = nextStep();
        SmartDashboard.putString("Localization Step", step);
        SmartDashboard.putNumber("Aim Check (deg)", Double.isNaN(aimCheckDeg) ? -999 : aimCheckDeg);
        SmartDashboard.putString("Aim Check", Double.isNaN(aimCheckDeg) ? "no hub tag in view"
            : Math.abs(aimCheckDeg) <= kAimCheckPassDeg ? "PASS" : "OFF BY " + Math.round(aimCheckDeg) + " deg");
        SmartDashboard.putString("Aim Check Tally", aimCheckGood + " good / " + aimCheckBad + " bad loops");
        DogLog.log("Localization/Step", step);
    }

    // ---------------- Test: Turret Sweep ----------------

    private record Sample(double turretDeg, double x, double y, double headingDiffDeg) {}

    /**
     * Robot ENABLED and sitting still, 1+ tags in view (2+ is better), 1-4 m away. Turns the turret
     * slowly from one end to the other and back, recording where the Limelight says the robot is.
     * The robot isn't moving, so a correct setup gives the same position the whole time.
     */
    public Command turretSweepTest() {
        List<Sample> samples = new ArrayList<>();
        double lo = Constants.Turret.kAimMarginDegrees;
        double hi = Constants.Turret.kEndDegrees - Constants.Turret.kAimMarginDegrees;
        double speed = 15.0; // deg/s, slow enough for vision
        double[] target = {lo};
        int[] dir = {1};

        Command sweep = Commands.run(() -> {
            target[0] += dir[0] * speed * 0.02;
            if (target[0] >= hi) { target[0] = hi; dir[0] = -1; }
            turret.aimAtAngle(target[0], 0.0);
            turret.setAimStatus("SWEEP TEST");
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
            if (mt1 != null && mt1.tagCount >= 2 && mt1.avgTagDist < 4.5
                    && Math.abs(turret.getVelocityDegPerSec()) < 40) {
                double gyro = drivetrain.getState().Pose.getRotation().getDegrees();
                samples.add(new Sample(turret.getAngleDegrees(), mt1.pose.getX(), mt1.pose.getY(),
                    ShotSolver.wrap180(mt1.pose.getRotation().getDegrees() - gyro)));
                DogLog.log("Localization/SweepSamplePose", mt1.pose);
                DogLog.log("Localization/SweepTurretDeg", turret.getAngleDegrees());
            }
        }, turret).until(() -> dir[0] == -1 && target[0] <= lo + 0.5);

        return Commands.sequence(
            Commands.runOnce(() -> {
                samples.clear();
                target[0] = Math.max(lo, turret.getAngleDegrees());
                dir[0] = 1;
                SmartDashboard.putString("Turret Sweep Result", "RUNNING... keep the robot still");
                report("Turret sweep test started");
            }),
            Commands.either(sweep, Commands.runOnce(() -> SmartDashboard.putString("Turret Sweep Result",
                    "NOT RUN: enable the robot and set the turret far end first")),
                () -> DriverStation.isEnabled() && turret.endMeasured()),
            Commands.runOnce(() -> {
                turret.stop();
                turret.clearAngleTarget();
                if (DriverStation.isEnabled()) {
                    String result = judge(samples);
                    SmartDashboard.putString("Turret Sweep Result", result);
                    report("Turret sweep: " + result);
                }
            })
        ).finallyDo(() -> { turret.stop(); turret.clearAngleTarget(); }).withName("Test: Turret Sweep");
    }

    /** Plain-English verdict from the sweep samples. */
    static String judge(List<Sample> s) {
        if (s.size() < 25) {
            return "NOT ENOUGH 2-TAG VIEWS (" + s.size() + "): put the robot 2-3 m from a hub face, turned so the"
                + " hub is off to its side (turret around the middle of its range), and run it again";
        }
        // Turret range covered while 2 tags were in view.
        double tMin = Double.MAX_VALUE, tMax = -Double.MAX_VALUE;
        for (Sample p : s) { tMin = Math.min(tMin, p.turretDeg()); tMax = Math.max(tMax, p.turretDeg()); }
        double range = tMax - tMin;

        // Robust spread: distance of each sample from the MEDIAN position; judge by the 80th
        // percentile so a few bad camera frames can't fail the test on their own.
        double[] xs = s.stream().mapToDouble(Sample::x).sorted().toArray();
        double[] ys = s.stream().mapToDouble(Sample::y).sorted().toArray();
        double mx = xs[xs.length / 2], my = ys[ys.length / 2];
        double[] d = s.stream().mapToDouble(p -> Math.hypot(p.x() - mx, p.y() - my)).sorted().toArray();
        double spread = d[(int) Math.floor(0.8 * (d.length - 1))];

        // Heading from vision minus gyro heading should NOT change as the turret turns. If it
        // changes about 2 degrees per degree of turret, the camera's turn direction is backwards.
        double mt = 0, mh = 0;
        for (Sample p : s) { mt += p.turretDeg(); mh += p.headingDiffDeg(); }
        mt /= s.size(); mh /= s.size();
        double num = 0, den = 0;
        for (Sample p : s) { num += (p.turretDeg() - mt) * (p.headingDiffDeg() - mh); den += Math.pow(p.turretDeg() - mt, 2); }
        double slope = den > 1e-6 ? num / den : 0;

        String nums = String.format(" [spread %.0f cm (80%%), heading drift %.2f deg/deg, turret %.0f-%.0f deg, %d samples]",
            spread * 100, slope, tMin, tMax, s.size());
        if (range < 20) {
            return "NOT ENOUGH TURRET RANGE: turn the robot so the hub is off to its side, run again." + nums;
        }
        if (Math.abs(slope) > 0.8) {
            return "FAIL: camera turns the wrong way. Flip Constants.Aim.kLimelightYawCCW, deploy, run again." + nums;
        }
        if (spread > 0.20) {
            return "FAIL: position moves as the turret turns. Send this line to the programmer (camera"
                + " offset or kLimelightSideIsRight)." + nums;
        }
        if (spread > 0.10) {
            return "OK-ISH: small wobble. Fine to continue." + nums;
        }
        return "PASS: position stays put while the turret turns." + nums;
    }

    private static void report(String msg) {
        System.out.println("LOCALIZATION | " + msg);
        DogLog.log("Localization/Report", msg);
    }
}
