package frc.robot.subsystems;

import java.io.File;
import java.io.FileWriter;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.List;
import java.util.Locale;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * SHOT MAP DAY guide. Walks the team through recording the shot table, one plain-English line at
 * a time ("Shot Map Step"), fills in good starting numbers at each new distance, and writes every
 * shot to a CSV file that anyone can download in a browser while connected to the robot:
 * http://10.56.69.2:5805/shot-log.csv (see Robot.java). Guide: tools/shot-map/.
 *
 * <p>It only reads and suggests; it never moves anything.
 */
public class ShotMapGuide extends SubsystemBase {
    /** Distances (m, turret center to hub center) to record, closest first. */
    public static final double[] kPlan = {1.5, 2.25, 3.0, 3.75, 4.5, 5.25};
    private static final double kCloseEnough = 0.25; // m
    private static final String kLogDir = "/home/lvuser/shotmap";

    private final AimSubsystem aim;
    private final ShooterSubsystem shooter;
    private final GoobaSubsystem hood;
    private double prefilledFor = Double.NaN;
    private int lastShots = 0, lastRecords = 0;

    public ShotMapGuide(AimSubsystem aim, ShooterSubsystem shooter, GoobaSubsystem hood) {
        this.aim = aim;
        this.shooter = shooter;
        this.hood = hood;
        new File(kLogDir).mkdirs();
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
        double next = nextDistance();

        // New target distance: start from the table's best guess so tuning starts close.
        if (aim.tuningMode() && !Double.isNaN(next) && !(next == prefilledFor)) {
            SmartDashboard.putNumber("Tune RPM", Math.round(aim.tableRpm(next) / 50.0) * 50.0);
            SmartDashboard.putNumber("Tune Hood (rot)", Math.round(aim.tableHood(next) * 100) / 100.0);
            prefilledFor = next;
            System.out.println(String.format(Locale.US, "SHOT MAP | next distance %.2f m: starting at %.0f RPM, hood %.2f",
                next, aim.tableRpm(next), aim.tableHood(next)));
        }

        // Text for people: 5 times a second (these decode the saved table: too slow for every loop).
        if (++textLoops % 10 == 0) {
            SmartDashboard.putString("Shot Map Step", step(next));
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

    private void writeRow(String event) {
        String row = String.format(Locale.US, "%s,%s,%.3f,%.0f,%.0f,%.0f,%.3f,%.3f,%.3f,%b,%.0f,%.3f,%.2f,%.2f,%.2f,%b%n",
                LocalDateTime.now().format(DateTimeFormatter.ISO_LOCAL_DATE_TIME), event,
                aim.solution().distanceMeters(), shooter.lastShotTargetRpm(), shooter.lastLaunchRpm(),
                shooter.lastDipRpm(), shooter.lastRecoverySec(), aim.targetHood(), hood.getPosition(),
                aim.tuningMode(), SmartDashboard.getNumber("Tune RPM", 0), SmartDashboard.getNumber("Tune Hood (rot)", 0),
                IndexSubsystem.forwardVolts(), SmartDashboard.getNumber("Uptake Forward Volts", 0),
                RobotController.getBatteryVoltage(), aim.poseUsable());
        writer.execute(() -> {
            File f = new File(kLogDir, "shot-log.csv");
            if (f.length() > 2_000_000) return; // never fill the roboRIO
            boolean header = !f.exists() || f.length() == 0;
            try (FileWriter w = new FileWriter(f, true)) {
                if (header) {
                    w.write("time,event,distance_m,target_rpm,launch_rpm,dip_rpm,recovery_s,hood_target,hood_actual,"
                        + "tuning_mode,tune_rpm,tune_hood,spindex_volts,uptake_volts,battery_v,pose_trusted\n");
                }
                w.write(row);
            } catch (Exception e) {
                System.out.println("SHOT MAP | could not write shot log: " + e.getMessage());
            }
        });
    }
}
