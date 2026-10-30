package frc.robot.subsystems;

import java.io.File;
import java.io.FileWriter;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.List;
import java.util.Locale;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * SHOT MAP DAY guide. Walks the team through recording the shot table, one plain-English line at
 * a time ("Shot Map Step"), fills in good starting numbers at each new distance, and writes every
 * shot to a CSV file that anyone can download in a browser while connected to the robot:
 * http://10.56.69.2:5805/shot-log.csv (see Robot.java). Guide: tools/shot-map/.
 *
 * <p>It only reads and suggests; it never moves anything.
 *
 * <p>FREE MAPPING (Oct 28 night, switch "Shot Map: Free Mapping", saved, ON by default): no plan,
 * no target distances. Park anywhere the position is known, stand still half a second, and it
 * fills Tune RPM / Tune Hood with what the table says for THIS spot; fire a volley, adjust, and
 * press Record Shot Point when it goes in (Record always saves at the current Aim Distance).
 * "Shot Map Step" shows the distance, how far off the hub's center line the robot is, balls
 * launched at this spot, and the biggest gap left in the table. OFF = the old guided plan.
 */
public class ShotMapGuide extends SubsystemBase {
    /** Distances (m, turret center to hub center) to record, closest first. */
    public static final double[] kPlan = {1.5, 2.25, 3.0, 3.75, 4.5, 5.25};
    private static final double kCloseEnough = 0.25; // m
    private static final String kLogDir = "/home/lvuser/shotmap";

    private final AimSubsystem aim;
    private final ShooterSubsystem shooter;
    private final GoobaSubsystem hood;
    private final CommandSwerveDrivetrain drivetrain;
    private final Goober turret;
    private double prefilledFor = Double.NaN;
    private int lastShots = 0, lastRecords = 0;

    private static final String kFreeKey = "Shot Map: Free Mapping";
    /** Free mapping refills Tune RPM/Hood only after the robot has moved at least this far (m). */
    static final double kFreeRefillMove = 0.4;
    private boolean freeSaved;
    private double stillSince = Double.NaN;
    private double spotDistance = Double.NaN;
    private int shotsAtSpotStart = 0;

    public ShotMapGuide(AimSubsystem aim, ShooterSubsystem shooter, GoobaSubsystem hood,
                        CommandSwerveDrivetrain drivetrain, Goober turret) {
        this.aim = aim;
        this.shooter = shooter;
        this.hood = hood;
        this.drivetrain = drivetrain;
        this.turret = turret;
        new File(kLogDir).mkdirs();
        Preferences.initBoolean(kFreeKey, true);
        freeSaved = Preferences.getBoolean(kFreeKey, true);
        SmartDashboard.putBoolean(kFreeKey, freeSaved);
    }

    private boolean freeMapping() {
        boolean on = SmartDashboard.getBoolean(kFreeKey, freeSaved);
        if (on != freeSaved) { freeSaved = on; Preferences.setBoolean(kFreeKey, on); }
        return on;
    }

    /** Degrees the turret pivot is off the hub's center line: + = driver's LEFT, - = right. */
    private double offCenterDegrees() {
        Translation2d pivot = aim.solution().pivotField();
        Translation2d hub = AimSubsystem.ourHub();
        if (pivot == null || hub == null) return 0.0;
        boolean red = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
        double fwd = red ? pivot.getX() - hub.getX() : hub.getX() - pivot.getX();
        double left = red ? hub.getY() - pivot.getY() : pivot.getY() - hub.getY();
        return Math.toDegrees(Math.atan2(left, Math.abs(fwd)));
    }

    /** "1.69 to 3.13 m": the widest stretch between table points from 1.5 to 5.5 m. */
    static String biggestGap(List<Double> distances) {
        java.util.ArrayList<Double> d = new java.util.ArrayList<>(distances);
        d.add(1.5); d.add(5.5);
        java.util.Collections.sort(d);
        double best = -1, a = 0, b = 0;
        for (int i = 1; i < d.size(); i++) {
            double lo = Math.max(1.5, d.get(i - 1)), hi = Math.min(5.5, d.get(i));
            if (hi - lo > best) { best = hi - lo; a = lo; b = hi; }
        }
        return String.format(Locale.US, "%.2f to %.2f m", a, b);
    }

    private String freeStep() {
        if (!aim.tuningMode()) return "FREE MAP A) Turn ON \"Shot Tuning Mode\"";
        if (!aim.isActive()) return "FREE MAP B) Press Operator RB: auto-aim ON";
        if (!aim.poseUsable()) return "FREE MAP C) Position unknown: robot center on a center-line mark, "
            + "type Plot Distance, press Set Pose: Plot Spot (or let the camera find it)";
        double d = aim.solution().distanceMeters();
        double off = offCenterDegrees();
        String where = String.format(Locale.US, "%.2f m (%.1f ft), %.0f deg %s of center", d, d / 0.3048,
            Math.abs(off), off >= 0 ? "left" : "right");
        if (Double.isNaN(spotDistance) || Math.abs(d - spotDistance) > kFreeRefillMove) {
            return "FREE MAP: stop anywhere 1.5 to 5.5 m out and hold still. Now " + where;
        }
        int launched = shooter.finishedShots() - shotsAtSpotStart;
        return String.format(Locale.US, "FREE MAP at %s. Start numbers filled in. Fire a volley (%d launched here). "
            + "Op D-pad: Up/Down hood 0.1, Right/Left RPM 50. 8 of 10 in? Op Y records (buzz). Biggest gap: %s",
            where, launched, biggestGap(aim.scoreTableDistances()));
    }

    /** The plan: the backup spots' exact distances plus the standard ones not too close to them. */
    private List<double[]> plan() { // {distance, spotIndex or -1}
        java.util.ArrayList<double[]> out = new java.util.ArrayList<>();
        BackupShots.Spot[] spots = BackupShots.Spot.values();
        for (int i = 0; i < spots.length; i++) out.add(new double[] {BackupShots.distance(spots[i]), i});
        for (double d : kPlan) {
            boolean near = out.stream().anyMatch(p -> p[1] >= 0 && Math.abs(p[0] - d) < 0.3);
            if (!near) out.add(new double[] {d, -1});
        }
        // Left and right trench are the same distance: keep one row for the distance itself.
        out.sort((a, b) -> Double.compare(a[0], b[0]));
        java.util.ArrayList<double[]> merged = new java.util.ArrayList<>();
        for (double[] p : out) {
            if (!merged.isEmpty() && Math.abs(merged.get(merged.size() - 1)[0] - p[0]) < 0.05) continue;
            merged.add(p);
        }
        return merged;
    }

    private static String label(double[] p) {
        if (p[1] < 0) return String.format(Locale.US, "%.2fm", p[0]);
        String name = BackupShots.Spot.values()[(int) p[1]] == BackupShots.Spot.MIDSHOT ? "Midshot" : "Trench";
        return String.format(Locale.US, "%s %.2fm", name, p[0]);
    }

    /** First planned distance with no recorded point near it, or NaN when all are done. */
    private double nextDistance() {
        List<Double> done = aim.recordedDistances();
        for (double[] p : plan()) {
            double d = p[0];
            boolean have = done.stream().anyMatch(r -> Math.abs(r - d) < kCloseEnough + 0.05);
            if (!have) return d;
        }
        return Double.NaN;
    }

    private String progress() {
        List<Double> done = aim.recordedDistances();
        StringBuilder sb = new StringBuilder();
        for (double[] p : plan()) {
            boolean have = done.stream().anyMatch(r -> Math.abs(r - p[0]) < kCloseEnough + 0.05);
            sb.append(have ? "[x] " : "[ ] ").append(label(p)).append("   ");
        }
        return sb.toString();
    }

    private String step(double next) {
        if (!aim.tuningMode()) return "A) Turn ON \"Shot Tuning Mode\" (toggle on this page)";
        if (!aim.isActive()) return "B) Press Operator RB: auto-aim ON (turret swings to the hub)";
        if (!aim.poseUsable()) return "C) Hold still 2-3 m in front of the hub until Vision Status says OK SET";
        if (Double.isNaN(next)) return "DONE! All distances recorded. Turn OFF Shot Tuning Mode. Tell Marcos.";
        double d = aim.solution().distanceMeters();
        if (Math.abs(d - next) > kCloseEnough) {
            return String.format(Locale.US, "Drive %s to %.2f m (now %.2f m, %.0f cm to go)",
                d < next ? "BACK, away from the hub," : "CLOSER to the hub", next, d, Math.abs(next - d) * 100);
        }
        return String.format(Locale.US, "At %.2f m: hold RT for 4 balls. Short = raise Tune RPM 100. Long = lower. "
            + "3 of 4 in? Press Record Shot Point", next);
    }

    @Override
    public void periodic() {
        boolean free = freeMapping();
        double next = free ? Double.NaN : nextDistance(); // decodes the saved table: only when used

        // Standing still? (free mapping waits for it before filling in numbers)
        var speeds = drivetrain.getState().Speeds;
        boolean still = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) < 0.1
            && Math.abs(speeds.omegaRadiansPerSecond) < 0.2;
        double now = Timer.getFPGATimestamp();
        if (!still) stillSince = Double.NaN;
        else if (Double.isNaN(stillSince)) stillSince = now;
        boolean settled = still && now - stillSince > 0.5;

        if (free) {
            // New spot (moved more than kFreeRefillMove since the last fill): start from the table here.
            double d = aim.solution().distanceMeters();
            if (aim.tuningMode() && aim.poseUsable() && settled
                    && (Double.isNaN(spotDistance) || Math.abs(d - spotDistance) > kFreeRefillMove)) {
                SmartDashboard.putNumber("Tune RPM", Math.round(aim.tableRpm(d) / 50.0) * 50.0);
                SmartDashboard.putNumber("Tune Hood (rot)", Math.round(aim.tableHood(d) * 100) / 100.0);
                spotDistance = d;
                shotsAtSpotStart = shooter.finishedShots();
                System.out.println(String.format(Locale.US, "SHOT MAP | free spot %.2f m, %.0f deg off center: starting at %.0f RPM, hood %.2f",
                    d, offCenterDegrees(), aim.tableRpm(d), aim.tableHood(d)));
            }
        } else if (aim.tuningMode() && !Double.isNaN(next) && !(next == prefilledFor)) {
            // New target distance: start from the table's best guess so tuning starts close.
            SmartDashboard.putNumber("Tune RPM", Math.round(aim.tableRpm(next) / 50.0) * 50.0);
            SmartDashboard.putNumber("Tune Hood (rot)", Math.round(aim.tableHood(next) * 100) / 100.0);
            prefilledFor = next;
            System.out.println(String.format(Locale.US, "SHOT MAP | next distance %.2f m: starting at %.0f RPM, hood %.2f",
                next, aim.tableRpm(next), aim.tableHood(next)));
        }

        // Text for people: 5 times a second (these decode the saved table: too slow for every loop).
        if (++textLoops % 10 == 0) {
            SmartDashboard.putString("Shot Map Step", free ? freeStep() : step(next));
            SmartDashboard.putString("Shot Map Progress", progress());
            SmartDashboard.putString("Shot Table Code", aim.shotTableCode());
        }

        // Shot log: one row per finished shot, one per Record press.
        if (shooter.finishedShots() != lastShots) {
            lastShots = shooter.finishedShots();
            writeRow("SHOT");
        }
        if (aim.recordCount() != lastRecords) {
            lastRecords = aim.recordCount();
            writeRow("RECORD");
        }
    }

    private int textLoops = 0;
    // The file is written on its own thread: opening a file on every ball (10 a second in a volley)
    // stalled the robot loop (audit Oct 27).
    private final java.util.concurrent.ExecutorService writer =
        java.util.concurrent.Executors.newSingleThreadExecutor(r -> {
            Thread t = new Thread(r, "shot-log");
            t.setDaemon(true);
            t.setPriority(Thread.MIN_PRIORITY);
            return t;
        });

    static final String kHeader = "time,event,distance_m,target_rpm,launch_rpm,dip_rpm,recovery_s,hood_target,hood_actual,"
        + "tuning_mode,tune_rpm,tune_hood,spindex_volts,uptake_volts,battery_v,pose_trusted,"
        + "x_m,y_m,heading_deg,turret_deg,turret_aim_error_deg,off_center_deg,robot_speed_mps";
    private volatile boolean headerChecked = false;

    private void writeRow(String event) {
        var pose = drivetrain.getState().Pose;
        var sp = drivetrain.getState().Speeds;
        // Oct 28 night: + where the robot was and which way it faced, so misses that depend on the
        // robot's rotation (turret angle) show up in the data.
        String tail = String.format(Locale.US, ",%.3f,%.3f,%.1f,%.1f,%.2f,%.1f,%.2f",
                pose.getX(), pose.getY(), pose.getRotation().getDegrees(), turret.getAngleDegrees(),
                turret.aimErrorDegrees(), offCenterDegrees(),
                Math.hypot(sp.vxMetersPerSecond, sp.vyMetersPerSecond));
        String row = String.format(Locale.US, "%s,%s,%.3f,%.0f,%.0f,%.0f,%.3f,%.3f,%.3f,%b,%.0f,%.3f,%.2f,%.2f,%.2f,%b",
                LocalDateTime.now().format(DateTimeFormatter.ISO_LOCAL_DATE_TIME), event,
                aim.solution().distanceMeters(), shooter.lastShotTargetRpm(), shooter.lastLaunchRpm(),
                shooter.lastDipRpm(), shooter.lastRecoverySec(), aim.targetHood(), hood.getPosition(),
                aim.tuningMode(), SmartDashboard.getNumber("Tune RPM", 0), SmartDashboard.getNumber("Tune Hood (rot)", 0),
                IndexSubsystem.forwardVolts(), SmartDashboard.getNumber("Uptake Forward Volts", 0),
                RobotController.getBatteryVoltage(), aim.poseUsable()) + tail + "\n";
        writer.execute(() -> {
            File f = new File(kLogDir, "shot-log.csv");
            if (!headerChecked) {
                headerChecked = true;
                // A file started by older code has fewer columns: keep it under its own name.
                try {
                    if (f.exists() && f.length() > 0) {
                        String first;
                        try (var lines = java.nio.file.Files.lines(f.toPath())) { first = lines.findFirst().orElse(""); }
                        if (!first.equals(kHeader)) {
                            File old = new File(kLogDir, "shot-log-before-" + LocalDateTime.now()
                                .format(DateTimeFormatter.ofPattern("yyyyMMdd-HHmmss")) + ".csv");
                            if (f.renameTo(old)) System.out.println("SHOT MAP | old shot log kept as " + old.getName());
                        }
                    }
                } catch (Exception e) {
                    System.out.println("SHOT MAP | could not check shot log header: " + e.getMessage());
                }
            }
            if (f.length() > 2_000_000) return; // never fill the roboRIO
            boolean header = !f.exists() || f.length() == 0;
            try (FileWriter w = new FileWriter(f, true)) {
                if (header) w.write(kHeader + "\n");
                w.write(row);
            } catch (Exception e) {
                System.out.println("SHOT MAP | could not write shot log: " + e.getMessage());
            }
        });
    }
}
