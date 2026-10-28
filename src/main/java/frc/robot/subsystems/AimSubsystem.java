package frc.robot.subsystems;

import java.util.Optional;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Aim;
import frc.robot.HubShiftTracker;
import frc.robot.aiming.ShotSolver;
import frc.robot.aiming.ShotTable;
import frc.robot.aiming.TargetPicker;

/**
 * The shot brain. Every loop, from where the robot is and how it moves:
 * <ul>
 *   <li>WHAT to shoot at (TargetPicker): our hub (SCORE), or a corner of our alliance zone (PASS)
 *       once the robot is past our hub. Operator Start cycles AUTO / SCORE / PASS.</li>
 *   <li>The turret angle, distance and turret rate, with shoot-on-the-move (ShotSolver).</li>
 *   <li>Shooter RPM and hood from the SCORE or PASS table at the (virtual) distance.</li>
 *   <li>Whether it's ready to fire (looser for passing), and whether a scoring shot would land
 *       in our hub while it's CLOSED (hub shifts, from FMS game data).</li>
 * </ul>
 * It never moves a motor itself (AimCommand and ShootCommand do), so nothing ever "requires" it.
 *
 * <p>Table tuning (no redeploy): "Shot Tuning Mode" ON, set "Tune RPM" / "Tune Hood (rot)", shoot
 * until it works, "Record Shot Point". It records into the table of the CURRENT target (SCORE or
 * PASS, dashboard "Shot Target"), saved on the roboRIO and used right away.
 */
public class AimSubsystem extends SubsystemBase {
    private static final String kPrefKey = "Shot Table Recorded";
    private static final String kPassPrefKey = "Pass Table Recorded";
    private static final String kClimbPrefKey = "Climb Table Recorded";
    private static final String kClimbSwitchKey = "Climb Shot Table";
    private static final String kPassFeedKey = "Pass Feed Volts";
    private static final String kHoldKey = "Hold Fire: Hub Closed";

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionLocalizer localizer;
    private final ShotSolver.Geometry geometry = new ShotSolver.Geometry(
        new Translation2d(Aim.kTurretPivotX, Aim.kTurretPivotY),
        Aim.kHomeYawDegrees, Constants.Turret.kEndDegrees, Constants.Turret.kAimMarginDegrees);
    private final TargetPicker picker = new TargetPicker(new TargetPicker.Field(
        Aim.kFieldLength, Aim.kFieldWidth, Aim.kBlueHubX, Aim.kHubY, Aim.kHubHalfSize,
        Aim.kScoreMaxRobotDepth, Aim.kTargetHysteresis, Aim.kPassDepth, Aim.kPassSide,
        Aim.kPassBackupDepth, Aim.kPassBackupSide, Aim.kPassObstaclePad,
        Aim.kPassMaxRange, Aim.kRelayDepth, Aim.kRelaySide));
    private static final String kRangeKey = "Pass Max Range (m)";

    private ShotTable table = ShotTable.defaults();
    private ShotTable passTable = ShotTable.passDefaults();
    // SHOOT WHILE CLIMBING: hanging on the tower lifts the shooter, so the same distance needs a
    // different RPM/hood. Its own table, used automatically while the elevator is up.
    private ShotTable climbTable = ShotTable.climbDefaults();
    private ShotSolver.Solution solution;
    private ShotSolver.Solution hubSolution; // our hub, no lead: where the camera should look in autos
    private TargetPicker.Target target;
    private TargetPicker.Mode mode = TargetPicker.Mode.AUTO;
    private double robotSpeed = 0;
    private boolean legalToScore = true;
    private boolean nearTrench = false;      // hood/Limelight in or heading into a trench
    private boolean trenchOverride = false;  // trench shot: hood allowed up while the robot stays put
    private Translation2d overrideAnchor = null;
    private String overrideNote = "";
    private boolean inTheirZone = false;
    private boolean active = false; // auto-aim (Operator RB / vacuum mode) or an auto is aiming
    private boolean firing = false; // ShootCommand is feeding balls right now
    private boolean vacuum = false; // vacuum mode: fires by itself, so it must know where it is
    private final Field2d field = new Field2d();
    private final SendableChooser<TargetPicker.Mode> modeChooser = new SendableChooser<>();

    public AimSubsystem(CommandSwerveDrivetrain drivetrain, VisionLocalizer localizer) {
        this.drivetrain = drivetrain;
        this.localizer = localizer;
        loadRecorded();
        SmartDashboard.putBoolean("Shot Tuning Mode", false);
        SmartDashboard.putNumber("Tune RPM", 4000);
        SmartDashboard.putNumber("Tune Hood (rot)", 1.0);
        SmartDashboard.putNumber("Tune TOF (s)", 0.9);
        Preferences.initBoolean(kHoldKey, true);
        Preferences.initDouble(kRangeKey, Aim.kPassMaxRange);
        Preferences.initBoolean(kClimbSwitchKey, true);
        Preferences.initDouble(kPassFeedKey, Aim.kPassFeedVolts);
        SmartDashboard.putNumber(kPassFeedKey, Preferences.getDouble(kPassFeedKey, Aim.kPassFeedVolts));
        SmartDashboard.putBoolean(kClimbSwitchKey, Preferences.getBoolean(kClimbSwitchKey, true));
        SmartDashboard.putNumber(kRangeKey, Preferences.getDouble(kRangeKey, Aim.kPassMaxRange));
        SmartDashboard.putBoolean(kHoldKey, Preferences.getBoolean(kHoldKey, true));
        modeChooser.setDefaultOption("AUTO (score near our hub, pass from past it)", TargetPicker.Mode.AUTO);
        modeChooser.addOption("SCORE only", TargetPicker.Mode.SCORE);
        modeChooser.addOption("PASS only", TargetPicker.Mode.PASS);
        SmartDashboard.putData("Shot Target Mode", modeChooser);
        SmartDashboard.putData("Field", field);
        solve();
    }

    private static boolean red() {
        Optional<Alliance> a = DriverStation.getAlliance();
        return a.isPresent() && a.get() == Alliance.Red;
    }

    /** Our hub's center: red or blue from the Driver Station (blue until it says). */
    public static Translation2d ourHub() {
        return new Translation2d(red() ? Aim.kRedHubX : Aim.kBlueHubX, Aim.kHubY);
    }

    // ---------- target mode (Operator Start cycles it; dashboard "Shot Target Mode" too) ----------

    private int describeLoops = 0;
    private TargetPicker.Mode buttonMode = null; // set by Operator Start, overrides the chooser

    public void cycleMode() {
        TargetPicker.Mode cur = currentMode();
        buttonMode = switch (cur) {
            case AUTO -> TargetPicker.Mode.SCORE;
            case SCORE -> TargetPicker.Mode.PASS;
            case PASS -> TargetPicker.Mode.AUTO;
        };
        System.out.println("AIM | Shot target mode: " + buttonMode);
        frc.robot.controls.Haptics.shortBuzzes(frc.robot.controls.Haptics.Pad.OPERATOR, buttonMode.ordinal() + 1);
    }

    private TargetPicker.Mode lastChooser = TargetPicker.Mode.AUTO;

    /** A match is starting: Operator Start's pick goes back to the dashboard chooser (AUTO). */
    public void resetTargetMode() {
        if (buttonMode != null) System.out.println("AIM | Shot target mode reset at match start");
        buttonMode = null;
    }

    public TargetPicker.Mode currentMode() {
        TargetPicker.Mode m = modeChooser.getSelected();
        if (m == null) m = TargetPicker.Mode.AUTO;
        if (m != lastChooser) { lastChooser = m; buttonMode = null; } // dashboard pick wins when changed
        return buttonMode != null ? buttonMode : m;
    }

    /** SCORE or PASS right now. */
    public TargetPicker.Kind targetKind() { return target.kind(); }
    public boolean passing() { return target.kind() == TargetPicker.Kind.PASS; }

    private ShotTable activeTable() { return passing() ? passTable : climbShot() ? climbTable : table; }

    /**
     * Shooting from the tower: HANGING (elevator went up, then pulled the robot up) and a SCORE
     * shot. Uses the CLIMB table. Dashboard switch "Climb Shot Table" (saved) turns it off (then
     * the normal table). While the elevator is up but the robot is still on the floor, the normal
     * table is used.
     */
    public boolean climbShot() {
        return !passing() && ElevatorSubsystem.isHanging() && SmartDashboard.getBoolean(kClimbSwitchKey, true);
    }

    /** Which table the shot uses right now: SCORE, CLIMB or PASS. */
    private String tableName() { return passing() ? "PASS" : climbShot() ? "CLIMB" : "SCORE"; }

    // ---------- solving ----------

    private double effectiveTof(double distance) {
        double t = rawTof(distance);
        double k = Aim.kDragK;
        return k <= 0 ? t : (1.0 - Math.exp(-k * t)) / k;
    }

    private double rawTof(double distance) {
        return tuningMode() ? SmartDashboard.getNumber("Tune TOF (s)", 0.9) : activeTable().timeOfFlight(distance);
    }

    private void solve() {
        var state = drivetrain.getState();
        Pose2d pose = state.Pose;
        // CTRE reports robot-relative speeds: turn them into field speeds.
        ChassisSpeeds fs = ChassisSpeeds.fromRobotRelativeSpeeds(state.Speeds, pose.getRotation());
        robotSpeed = Math.hypot(fs.vxMetersPerSecond, fs.vyMetersPerSecond);
        Translation2d pivot = pose.getTranslation().plus(geometry.turretPivot().rotateBy(pose.getRotation()));
        mode = currentMode();
        // G407 is judged where the robot is when the ball LEAVES: check now and the predicted spot.
        Translation2d atLaunch = pose.getTranslation().plus(new Translation2d(
            fs.vxMetersPerSecond * Aim.kLaunchLookahead, fs.vyMetersPerSecond * Aim.kLaunchLookahead));
        legalToScore = !poseUsable() || (picker.legalToScore(pose.getTranslation(), red())
            && picker.legalToScore(atLaunch, red()));
        // TRENCH GUARD: the hood/Limelight (on the turret) in or heading into a trench = hood DOWN.
        nearTrench = poseUsable() && picker.trenchDanger(pivot, fs.vxMetersPerSecond * Aim.kTrenchLookahead,
            fs.vyMetersPerSecond * Aim.kTrenchLookahead, Aim.kTrenchReach, Aim.kTrenchOpeningY);
        // The anchor is taken one loop after the press (a backup-shot button resets the pose first).
        if (trenchOverride && overrideAnchor == null) overrideAnchor = pose.getTranslation();
        else if (trenchOverride && (!poseUsable()
                || pose.getTranslation().getDistance(overrideAnchor) > Aim.kTrenchOverrideMove
                || robotSpeed > Aim.kTrenchOverrideSpeed)) {
            trenchOverride = false;
            overrideNote = "trench-shot override ended (robot moved)";
            System.out.println("TRENCH | " + overrideNote);
        }
        GoobaSubsystem.setTrenchForceDown(nearTrench && !trenchOverride);
        // Where the launcher will be when the balls now in the robot have left (lane check).
        Translation2d pivotAhead = pivot.plus(new Translation2d(
            fs.vxMetersPerSecond * Aim.kLaneLookahead, fs.vyMetersPerSecond * Aim.kLaneLookahead));
        double range = passRange();
        inTheirZone = poseUsable() && picker.inTheirZone(pose.getTranslation(), red(), Aim.kAllianceZoneDepth);
        // No position: passing is impossible, aim falls back to our hub tag (scoring).
        if (!poseUsable()) {
            target = new TargetPicker.Target(TargetPicker.Kind.SCORE, ourHub(), "HUB (tag only)");
        } else if (firing && target != null && target.kind() == TargetPicker.Kind.PASS
                && mode != TargetPicker.Mode.SCORE && target.laneClear()
                && picker.laneClear(pivot, pivotAhead, target.point(), red())
                && TargetPicker.inRange(pivot, pivotAhead, target.point(), range)) {
            // Mid-volley while passing: keep the same landing point so the burst doesn't jump
            // corners (1678 froze its target while firing), as long as its lane stays clear from
            // here and from where the robot will be in kLaneLookahead. Once a hub gets in the way
            // it picks again: another clear lane, or hold fire until one opens.
        } else {
            target = picker.pick(pose.getTranslation(), pivot, pivotAhead, red(), mode, range);
        }
        // During an agitation shimmy the robot rocks a centimeter or two: no motion lead then.
        boolean lead = Aim.kShootWhileMoving && !Agitator.shimmying();
        hubSolution = ShotSolver.solve(pose, fs.vxMetersPerSecond, fs.vyMetersPerSecond,
            state.Speeds.omegaRadiansPerSecond, ourHub(), geometry, this::effectiveTof, -1.0);
        solution = ShotSolver.solve(pose, fs.vxMetersPerSecond, fs.vyMetersPerSecond,
            state.Speeds.omegaRadiansPerSecond, target.point(), geometry, this::effectiveTof,
            lead ? Aim.kReleaseLatency : -1.0);
    }

    // ---------- what the commands ask ----------

    public ShotSolver.Solution solution() { return solution; }

    /** Turret angle/rate to look at OUR HUB with no shoot-on-the-move lead (camera aiming in autos). */
    public ShotSolver.Solution hubSolution() { return hubSolution; }

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
        return poseUsable() ? activeTable().rpm(solution.distanceMeters()) : Constants.Shooter.kFastTargetRPM;
    }

    public double targetHood() {
        double h = tuningMode() ? SmartDashboard.getNumber("Tune Hood (rot)", 1.0) : activeTable().hood(solution.distanceMeters());
        // Trench guard: the Limelight on the hood is taller than the trench bar with the hood up.
        return GoobaSubsystem.trenchForceDown() ? Constants.Gooba.kBottomPosition : h;
    }

    public boolean nearTrench() { return nearTrench; }

    /** The shot needs the hood up, but the trench guard holds it down: don't fire. */
    private boolean trenchBlocksShot() {
        if (!GoobaSubsystem.trenchForceDown() || !Constants.Gooba.kHoodEnabled) return false;
        double want = tuningMode() ? SmartDashboard.getNumber("Tune Hood (rot)", 1.0) : activeTable().hood(solution.distanceMeters());
        return want > Constants.Gooba.kBottomPosition + hoodTolerance();
    }

    /**
     * TRENCH SHOT (Operator R3, and the trench backup-shot buttons): let the hood go up while the
     * robot stays put next to the trench. Refused if the turret is actually under a trench bar.
     * Ends by itself once the robot moves 0.15 m (6 in) or drives faster than 0.3 m/s.
     */
    public void engageTrenchOverride() {
        var pose = drivetrain.getState().Pose;
        Translation2d pivot = pose.getTranslation().plus(geometry.turretPivot().rotateBy(pose.getRotation()));
        if (!poseUsable()) {
            overrideNote = "trench-shot override REFUSED: position unknown";
        } else if (picker.trenchDanger(pivot, 0, 0, 0.10, Aim.kTrenchOpeningY)) {
            overrideNote = "trench-shot override REFUSED: the hood is UNDER the trench bar";
        } else {
            trenchOverride = true;
            overrideAnchor = null; // set on the next loop, after any pose reset
            overrideNote = "trench-shot override ON: hood may go up (stay put)";
        }
        System.out.println("TRENCH | " + overrideNote);
    }

    public Command trenchOverrideCommand() {
        return Commands.runOnce(this::engageTrenchOverride).withName("Trench shot override");
    }

    /** Robot is inside the OPPONENT's alliance zone (shuttling from there). */
    public boolean inTheirZone() { return inTheirZone; }

    /** Spindexer + uptake volts while PASSING: dashboard "Pass Feed Volts" (saved, default 12 = max). */
    public double passFeedVolts() {
        double v = MathUtil.clamp(SmartDashboard.getNumber(kPassFeedKey, Aim.kPassFeedVolts), 0.0, 12.0);
        if (v != Preferences.getDouble(kPassFeedKey, Aim.kPassFeedVolts)) Preferences.setDouble(kPassFeedKey, v);
        return v;
    }

    /** Longest pass (m): dashboard "Pass Max Range (m)", saved on the roboRIO. */
    private double passRange() {
        double r = MathUtil.clamp(SmartDashboard.getNumber(kRangeKey, Aim.kPassMaxRange), 3.0, 20.0);
        if (r != Preferences.getDouble(kRangeKey, Aim.kPassMaxRange)) Preferences.setDouble(kRangeKey, r);
        return r;
    }

    /** Turret feedforward rate (deg/s clockwise) for the current target, 0 without position. */
    public double turretRate() { return poseUsable() ? solution.turretRateDegPerSec() : 0.0; }

    public boolean isActive() { return active; }
    /** Vacuum mode on/off (it auto-fires, so it never uses the tag-only fallback). */
    public void setVacuum(boolean on) { vacuum = on; }
    public boolean vacuum() { return vacuum; }

    /** ShootCommand reports whether it is feeding (freezes the pass target for the volley). */
    public void setFiring(boolean on) { firing = on; }
    /** ShootCommand is feeding balls right now. */
    public boolean isFiring() { return firing; }
    public void setActive(boolean on) { active = on; }

    private volatile boolean shotWanted = true;
    /**
     * A shot is wanted now: Operator RT/LT held, vacuum/dump on, firing, or not teleop (autos). With
     * auto-aim on all teleop, the hood stays DOWN and the robot never turns itself until this is true.
     */
    public boolean shotWanted() { return shotWanted; }
    public void setShotWanted(boolean w) { shotWanted = w; }

    /** Aim tolerance (deg) for the current target and distance. */
    public double aimToleranceDegrees() {
        double margin = passing() ? Aim.kPassAimMargin : Aim.kScoreAimMargin;
        double deg = Math.toDegrees(Math.atan2(margin, Math.max(solution.distanceMeters(), 0.5)));
        return Math.max(Aim.kTurretToleranceDegrees, deg);
    }

    private boolean turretOnTarget(Goober turret) {
        return turret.aimTracking(aimToleranceDegrees(), turretRate(), Aim.kTurretRateTolerance);
    }

    private boolean speedOk() {
        return robotSpeed <= (passing() ? Aim.kMaxPassSpeed : Aim.kMaxScoreSpeed);
    }

    /** A scoring shot fired now would land in our hub while it's CLOSED (FMS data needed). */
    public boolean hubClosedForShot() {
        if (passing() || !SmartDashboard.getBoolean(kHoldKey, true)) return false;
        // Judge the hub a little BEFORE the ball lands (tof - margin): if it's open by then it is
        // open when the ball lands (the match clock is approximate). Closing is covered by the
        // tracker's grace (the hub counts FUEL 3 s after it deactivates).
        double ahead = Math.max(0.0, rawTof(solution.distanceMeters()) - Aim.kHubArrivalMargin);
        return HubShiftTracker.ourHubStateAt(ahead) == HubShiftTracker.HubState.CLOSED;
    }

    /**
     * All clear to feed balls: aiming, position known, target in reach, turret on target (and
     * tracking it at the right speed while moving), robot not too fast, shooter at speed, hood in
     * place, and (scoring) our hub open when the ball gets there.
     */
    public boolean readyToFire(Goober turret, ShooterSubsystem shooter, GoobaSubsystem hood) {
        if (!active) return false;
        if (!poseUsable()) {
            // Vacuum fires by itself: without a position it can't check the G407 zone line.
            if (vacuum) return false;
            // Tag-only fallback: turret locked on the hub tag, shooter at speed (hood follows ty).
            return turret.targetLocked() && rpmReady(shooter);
        }
        if (!solution.reachable() || !turretOnTarget(turret) || !speedOk()) return false;
        if (!passing() && !legalToScore) return false; // G407: not inside our alliance zone
        if (passing() && !target.laneClear()) return false; // a hub is in every pass lane
        if (!rpmReady(shooter) || hubClosedForShot() || trenchBlocksShot()) return false;
        return !Constants.Gooba.kHoodEnabled || Math.abs(hood.getPosition() - targetHood()) <= hoodTolerance();
    }

    private double hoodTolerance() { return passing() ? Aim.kPassHoodTolerance : Aim.kHoodTolerance; }

    /**
     * Once firing has started, keep feeding through the normal RPM dip each ball causes. Feeding
     * only stops if aim is switched off, the target goes behind the turret, the turret falls well
     * off target, the robot gets too fast, or (scoring) our hub is about to be closed.
     */
    public boolean keepFiring(Goober turret) {
        if (!active) return false;
        if (!poseUsable()) return !vacuum;
        double keep = Math.max(Aim.kKeepFiringDegrees, aimToleranceDegrees() * 1.5);
        return solution.reachable() && Math.abs(turret.aimErrorDegrees()) <= keep
            && speedOk() && !hubClosedForShot() && !trenchBlocksShot()
            && (passing() ? target.laneClear() : legalToScore);
    }

    /** Shooter within tolerance of its target (looser for passing). */
    public boolean rpmReady(ShooterSubsystem shooter) {
        double t = targetRpm();
        double tol = passing() ? Aim.kPassRpmTolerance
            : Math.max(Aim.kRpmTolerance, Aim.kRpmTolerancePct * Math.abs(t));
        return Math.abs(shooter.getCurrentRpm() - t) <= tol;
    }

    /** Why it isn't ready (dashboard "Shot Ready" text). */
    public String readyText(Goober turret, ShooterSubsystem shooter, GoobaSubsystem hood) {
        if (!active) return "aim off (Operator RB)";
        if (!poseUsable()) {
            if (vacuum) return (frc.robot.commands.VacuumCommand.dumping() ? "DUMP" : "VACUUM") + " holds: position unknown (face the hub until Vision Status says OK)";
            return turret.targetLocked() ? "tag-only: shooter spinning up" : "tag-only: turret not on hub tag";
        }
        String what = passing() ? (TargetPicker.isRelay(target) ? "SHUTTLE" : "PASS") : climbShot() ? "CLIMB SCORE" : "SCORE";
        if (!passing() && !legalToScore) return "SCORE: outside our alliance zone (G407 foul): drive in or PASS";
        if (passing() && !target.laneClear()) return (TargetPicker.isRelay(target) ? "SHUTTLE" : "PASS")
            + ": a hub is (or is about to be) in every lane: hold, move sideways";
        if (!solution.reachable()) return what + ": target behind turret, turning robot";
        if (!speedOk()) return String.format(java.util.Locale.US, "%s: slow down (%.1f m/s, max %.1f)",
            what, robotSpeed, passing() ? Aim.kMaxPassSpeed : Aim.kMaxScoreSpeed);
        if (trenchBlocksShot()) return what + ": TRENCH, hood held down (Operator R3 = trench shot)";
        if (!turretOnTarget(turret)) return what + ": turret turning";
        if (!rpmReady(shooter)) return what + ": shooter spinning up";
        if (Constants.Gooba.kHoodEnabled && Math.abs(hood.getPosition() - targetHood()) > hoodTolerance()) return what + ": hood moving";
        if (hubClosedForShot()) {
            double s = HubShiftTracker.secondsUntilChange(HubShiftTracker.matchTime());
            return s > 0 ? String.format(java.util.Locale.US, "HUB CLOSED: opens in %.0f s (pass instead?)", s)
                : "HUB CLOSED";
        }
        return what + " READY";
    }

    // ---------- shot table recording ----------

    private void loadRecorded() {
        table = ShotTable.defaults();
        for (ShotTable.Point p : ShotTable.decode(Preferences.getString(kPrefKey, ""))) table.put(p);
        passTable = ShotTable.passDefaults();
        for (ShotTable.Point p : ShotTable.decode(Preferences.getString(kPassPrefKey, ""))) passTable.put(p);
        climbTable = ShotTable.climbDefaults();
        for (ShotTable.Point p : ShotTable.decode(Preferences.getString(kClimbPrefKey, ""))) climbTable.put(p);
    }

    /** Times "Record Shot Point" was pressed since boot (the Shot Map guide watches this). */
    private int recordCount = 0;
    public int recordCount() { return recordCount; }

    /** The SCORE table's current values at a distance (code points + recorded points). */
    public double tableRpm(double d) { return table.rpm(d); }
    public double tableHood(double d) { return table.hood(d); }

    /** SCORE distances (m) recorded on this robot so far. */
    public java.util.List<Double> recordedDistances() {
        return recordedPoints(kPrefKey).stream().map(ShotTable.Point::distance).toList();
    }

    /** Java lines for ShotTable.defaults() / passDefaults(), to paste the recorded tables into code. */
    public String shotTableCode() {
        return code(recordedPoints(kPrefKey)) + (recordedPoints(kPassPrefKey).isEmpty() ? ""
            : " | PASS: " + code(recordedPoints(kPassPrefKey)))
            + (recordedPoints(kClimbPrefKey).isEmpty() ? "" : " | CLIMB: " + code(recordedPoints(kClimbPrefKey)));
    }

    private static String code(java.util.List<ShotTable.Point> pts) {
        StringBuilder sb = new StringBuilder();
        for (ShotTable.Point p : pts) {
            sb.append(String.format(java.util.Locale.US, "t.put(new Point(%.2f, %.0f, %.3f, %.2f)); ",
                p.distance(), p.rpm(), p.hood(), p.tof()));
        }
        return sb.toString();
    }

    private java.util.List<ShotTable.Point> recordedPoints(String key) {
        return new java.util.ArrayList<>(ShotTable.decode(Preferences.getString(key, "")));
    }

    /** "Record Shot Point": saves the Tune values at the current distance, into the current target's table. */
    public Command recordPointCommand() {
        return runOnce(() -> {
            String key = passing() ? kPassPrefKey : climbShot() ? kClimbPrefKey : kPrefKey;
            double d = solution.distanceMeters();
            var pts = recordedPoints(key);
            pts.removeIf(p -> Math.abs(p.distance() - d) < ShotTable.kSameDistance);
            pts.add(new ShotTable.Point(d,
                SmartDashboard.getNumber("Tune RPM", 4000),
                SmartDashboard.getNumber("Tune Hood (rot)", 1.0),
                SmartDashboard.getNumber("Tune TOF (s)", 0.9)));
            pts.sort((a, b) -> Double.compare(a.distance(), b.distance()));
            Preferences.setString(key, ShotTable.encode(pts));
            loadRecorded();
            recordCount++;
            String which = passing() ? "PASS" : climbShot() ? "CLIMB" : "SHOT";
            System.out.println(which + " POINT RECORDED at " + String.format(java.util.Locale.US, "%.2f m", d));
            System.out.println(which + " TABLE | " + ShotTable.encode(pts));
            System.out.println("SHOT TABLE CODE | " + shotTableCode());
        }).ignoringDisable(true).withName("Record Shot Point");
    }

    /** "Clear Recorded Shot Points": back to the code's SCORE table (pass table: its own button). */
    public Command clearRecordedCommand() {
        return runOnce(() -> {
            Preferences.setString(kPrefKey, "");
            loadRecorded();
        }).ignoringDisable(true).withName("Clear Recorded Shot Points");
    }

    public Command clearRecordedClimbCommand() {
        return runOnce(() -> {
            Preferences.setString(kClimbPrefKey, "");
            loadRecorded();
        }).ignoringDisable(true).withName("Clear Recorded Climb Points");
    }

    public Command clearRecordedPassCommand() {
        return runOnce(() -> {
            Preferences.setString(kPassPrefKey, "");
            loadRecorded();
        }).ignoringDisable(true).withName("Clear Recorded Pass Points");
    }

    /** Operator Start: cycle AUTO / SCORE / PASS. */
    public Command cycleModeCommand() {
        return Commands.runOnce(this::cycleMode).ignoringDisable(true).withName("Cycle shot target");
    }

    @Override
    public void periodic() {
        boolean hold = SmartDashboard.getBoolean(kHoldKey, true);
        if (hold != Preferences.getBoolean(kHoldKey, true)) Preferences.setBoolean(kHoldKey, hold);
        solve();
        Pose2d pose = drivetrain.getState().Pose;
        field.setRobotPose(pose);
        field.getObject("Hub").setPose(new Pose2d(ourHub(), Rotation2d.kZero));
        field.getObject("Target").setPose(new Pose2d(target.point(), Rotation2d.kZero));
        field.getObject("Aim Point").setPose(new Pose2d(solution.aimPoint(), Rotation2d.kZero));

        SmartDashboard.putString("Shot Target", tableName() + " (" + currentMode() + "): " + target.name()
            + (inTheirZone ? " [in THEIR zone]" : ""));
        SmartDashboard.putBoolean("Pass Lane Clear", !passing() || target.laneClear());
        SmartDashboard.putNumber("Aim Distance (m)", solution.distanceMeters());
        SmartDashboard.putNumber("Aim Turret Angle (deg)", solution.turretDegrees());
        SmartDashboard.putNumber("Aim Turret Rate (deg per s)", solution.turretRateDegPerSec());
        SmartDashboard.putBoolean("Aim Hub In Reach", solution.reachable());
        SmartDashboard.putNumber("Aim Robot Turn Needed (deg)", Math.toDegrees(solution.headingErrorRad()));
        SmartDashboard.putNumber("Aim Shooter RPM", targetRpm());
        SmartDashboard.putNumber("Aim Hood (rot)", targetHood());
        SmartDashboard.putNumber("Aim Tolerance (deg)", aimToleranceDegrees());
        SmartDashboard.putNumber("Robot Speed (m per s)", robotSpeed);
        SmartDashboard.putBoolean("Near Trench (hood capped)", GoobaSubsystem.trenchForceDown());
        SmartDashboard.putString("Trench Guard", !poseUsable() ? "OFF: position unknown, keep the hood DOWN by hand"
            : GoobaSubsystem.trenchForceDown() ? "HOOD HELD DOWN (trench)"
            : trenchOverride ? "TRENCH SHOT: hood free, stay put" : nearTrench ? "near trench" : "clear");
        SmartDashboard.putString("Trench Override", overrideNote);
        SmartDashboard.putBoolean("Aim Active", active);
        HubShiftTracker.HubState hub = HubShiftTracker.ourHubStateAt(0);
        SmartDashboard.putString("Our Hub", hub == HubShiftTracker.HubState.UNKNOWN ? "unknown (no FMS)"
            : HubShiftTracker.isOurHubOpen() ? "OPEN" : "CLOSED");
        SmartDashboard.putString("Aim Geometry", Aim.kGeometryMeasured ? "measured" : "AIM GEOMETRY NOT MEASURED");
        if (++describeLoops % 25 == 0) { // twice a second is plenty for these long strings (loop time)
            SmartDashboard.putString("Shot Table", table.describe());
            SmartDashboard.putString("Pass Table", passTable.describe());
            SmartDashboard.putString("Climb Table", climbTable.describe());
        }
        boolean climbOn = SmartDashboard.getBoolean(kClimbSwitchKey, true);
        if (climbOn != Preferences.getBoolean(kClimbSwitchKey, true)) Preferences.setBoolean(kClimbSwitchKey, climbOn);
        DogLog.log("Aim/Distance", solution.distanceMeters());
        DogLog.log("Aim/TurretTarget", solution.turretDegrees());
        DogLog.log("Aim/TurretRate", solution.turretRateDegPerSec());
        DogLog.log("Aim/Reachable", solution.reachable());
        DogLog.log("Aim/Passing", passing());
        DogLog.log("Aim/LaneClear", !passing() || target.laneClear());
        DogLog.log("Aim/Shuttle", passing() && TargetPicker.isRelay(target));
        DogLog.log("Aim/Target", new Pose2d(target.point(), Rotation2d.kZero));
        DogLog.log("Aim/AimPoint", new Pose2d(solution.aimPoint(), Rotation2d.kZero));
    }
}
