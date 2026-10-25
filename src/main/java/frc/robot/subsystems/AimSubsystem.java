package frc.robot.subsystems;

import java.util.Optional;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Aim;
import frc.robot.aiming.ShotSolver;
import frc.robot.aiming.ShotTable;

/**
 * The shot brain for shooting from anywhere. Every loop it works out, from where the robot is:
 * the turret angle to our hub, the distance, and from the shot table the shooter speed and hood
 * position. It never moves a motor itself (AimCommand and ShootCommand do), so nothing ever
 * "requires" it.
 *
 * <p>Shot table tuning (no redeploy): turn on "Shot Tuning Mode", type "Tune RPM" and
 * "Tune Hood (rot)", shoot until it goes in, press "Record Shot Point". The point is saved on the
 * roboRIO and used right away. "Clear Recorded Shot Points" goes back to the code's table.
 */
public class AimSubsystem extends SubsystemBase {
    private static final String kPrefKey = "Shot Table Recorded";

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionLocalizer localizer;
    private final ShotSolver.Geometry geometry = new ShotSolver.Geometry(
        new Translation2d(Aim.kTurretPivotX, Aim.kTurretPivotY),
        Aim.kHomeYawDegrees, Constants.Turret.kEndDegrees, Constants.Turret.kAimMarginDegrees);

    private ShotTable table = ShotTable.defaults();
    private ShotSolver.Solution solution;
    private boolean active = false; // auto-aim (Operator RB) or an auto is aiming
    private final Field2d field = new Field2d();

    public AimSubsystem(CommandSwerveDrivetrain drivetrain, VisionLocalizer localizer) {
        this.drivetrain = drivetrain;
        this.localizer = localizer;
        loadRecorded();
        SmartDashboard.putBoolean("Shot Tuning Mode", false);
        SmartDashboard.putNumber("Tune RPM", 4000);
        SmartDashboard.putNumber("Tune Hood (rot)", 1.0);
        SmartDashboard.putNumber("Tune TOF (s)", 0.9);
        SmartDashboard.putData("Field", field);
        solve();
    }

    /** Our hub's center: red or blue from the Driver Station (blue until it says). */
    public static Translation2d ourHub() {
        Optional<Alliance> a = DriverStation.getAlliance();
        boolean red = a.isPresent() && a.get() == Alliance.Red;
        return new Translation2d(red ? Aim.kRedHubX : Aim.kBlueHubX, Aim.kHubY);
    }

    private void solve() {
        var state = drivetrain.getState();
        Pose2d pose = state.Pose;
        // CTRE reports robot-relative speeds: turn them into field speeds.
        ChassisSpeeds field = ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, pose.getRotation());
        solution = ShotSolver.solve(pose, field.vxMetersPerSecond, field.vyMetersPerSecond, ourHub(),
            geometry, this::timeOfFlight, Aim.kShootWhileMoving);
    }

    // ---------- what the commands ask ----------

    public ShotSolver.Solution solution() { return solution; }

    /** Robot's position can be used for aiming (otherwise aim falls back to the hub tag). */
    public boolean poseUsable() {
        // Until the turret/camera geometry is measured, aiming from position would be wrong:
        // auto-aim then works exactly like before (follows our hub tag).
        return Aim.kGeometryMeasured && localizer.poseTrusted();
    }

    public boolean tuningMode() { return SmartDashboard.getBoolean("Shot Tuning Mode", false); }

    public double targetRpm() {
        if (tuningMode()) return SmartDashboard.getNumber("Tune RPM", 4000);
        // Position unknown: the distance is meaningless, use the fast preset (tag-only fallback).
        return poseUsable() ? table.rpm(solution.distanceMeters()) : Constants.Shooter.kFastTargetRPM;
    }

    public double targetHood() {
        return tuningMode() ? SmartDashboard.getNumber("Tune Hood (rot)", 1.0) : table.hood(solution.distanceMeters());
    }

    private double timeOfFlight(double distance) {
        return tuningMode() ? SmartDashboard.getNumber("Tune TOF (s)", 0.9) : table.timeOfFlight(distance);
    }

    public boolean isActive() { return active; }
    public void setActive(boolean on) { active = on; }

    /**
     * All clear to feed balls: aiming, position known, hub in reach, turret on target, shooter at
     * speed, hood in place.
     */
    public boolean readyToFire(Goober turret, ShooterSubsystem shooter, GoobaSubsystem hood) {
        if (!active) return false;
        if (!poseUsable()) {
            // Tag-only fallback: turret locked on the hub tag, shooter at speed (hood follows ty).
            return turret.targetLocked() && Math.abs(shooter.getCurrentRpm() - targetRpm()) <= Aim.kRpmTolerance;
        }
        if (!solution.reachable() || !turret.aimLocked()) return false;
        if (Math.abs(shooter.getCurrentRpm() - targetRpm()) > Aim.kRpmTolerance) return false;
        return !Constants.Gooba.kHoodEnabled || Math.abs(hood.getPosition() - targetHood()) <= Aim.kHoodTolerance;
    }

    /** Why it isn't ready (dashboard "Shot Ready" text). */
    public String readyText(Goober turret, ShooterSubsystem shooter, GoobaSubsystem hood) {
        if (!active) return "aim off (Operator RB)";
        if (!poseUsable()) return turret.targetLocked() ? "tag-only: shooter spinning up" : "tag-only: turret not on hub tag";
        if (!solution.reachable()) return "hub behind turret: turning robot";
        if (!turret.aimLocked()) return "turret turning";
        if (Math.abs(shooter.getCurrentRpm() - targetRpm()) > Aim.kRpmTolerance) return "shooter spinning up";
        if (Constants.Gooba.kHoodEnabled && Math.abs(hood.getPosition() - targetHood()) > Aim.kHoodTolerance) return "hood moving";
        return "READY";
    }

    // ---------- shot table recording ----------

    private void loadRecorded() {
        table = ShotTable.defaults();
        for (ShotTable.Point p : ShotTable.decode(Preferences.getString(kPrefKey, ""))) {
            table.put(p);
        }
    }

    private java.util.List<ShotTable.Point> recordedPoints() {
        return new java.util.ArrayList<>(ShotTable.decode(Preferences.getString(kPrefKey, "")));
    }

    /** "Record Shot Point": saves the Tune values at the current distance. */
    public Command recordPointCommand() {
        return runOnce(() -> {
            double d = solution.distanceMeters();
            var pts = recordedPoints();
            pts.removeIf(p -> Math.abs(p.distance() - d) < ShotTable.kSameDistance);
            pts.add(new ShotTable.Point(d,
                SmartDashboard.getNumber("Tune RPM", 4000),
                SmartDashboard.getNumber("Tune Hood (rot)", 1.0),
                SmartDashboard.getNumber("Tune TOF (s)", 0.9)));
            pts.sort((a, b) -> Double.compare(a.distance(), b.distance()));
            Preferences.setString(kPrefKey, ShotTable.encode(pts));
            loadRecorded();
            System.out.println("SHOT POINT RECORDED: " + ShotTable.encode(java.util.List.of(pts.stream()
                .filter(p -> p.distance() == d).findFirst().get())));
        }).ignoringDisable(true).withName("Record Shot Point");
    }

    /** "Clear Recorded Shot Points": back to the code's table. */
    public Command clearRecordedCommand() {
        return runOnce(() -> {
            Preferences.setString(kPrefKey, "");
            loadRecorded();
        }).ignoringDisable(true).withName("Clear Recorded Shot Points");
    }

    @Override
    public void periodic() {
        solve();
        Pose2d pose = drivetrain.getState().Pose;
        field.setRobotPose(pose);
        Translation2d hub = ourHub();
        field.getObject("Hub").setPose(new Pose2d(hub, Rotation2d.kZero));
        field.getObject("Aim Point").setPose(new Pose2d(solution.aimPoint(), Rotation2d.kZero));

        SmartDashboard.putNumber("Aim Distance (m)", solution.distanceMeters());
        SmartDashboard.putNumber("Aim Turret Angle (deg)", solution.turretDegrees());
        SmartDashboard.putBoolean("Aim Hub In Reach", solution.reachable());
        SmartDashboard.putNumber("Aim Robot Turn Needed (deg)", Math.toDegrees(solution.headingErrorRad()));
        SmartDashboard.putNumber("Aim Shooter RPM", targetRpm());
        SmartDashboard.putNumber("Aim Hood (rot)", targetHood());
        SmartDashboard.putBoolean("Aim Active", active);
        SmartDashboard.putString("Aim Geometry", Aim.kGeometryMeasured ? "measured" : "AIM GEOMETRY NOT MEASURED");
        SmartDashboard.putString("Shot Table", table.describe());
        DogLog.log("Aim/Distance", solution.distanceMeters());
        DogLog.log("Aim/TurretTarget", solution.turretDegrees());
        DogLog.log("Aim/Reachable", solution.reachable());
        DogLog.log("Aim/AimPoint", new Pose2d(solution.aimPoint(), Rotation2d.kZero));
    }
}
