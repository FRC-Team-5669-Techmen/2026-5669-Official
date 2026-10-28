package frc.robot.subsystems;

import java.io.File;
import java.io.FileWriter;
import java.nio.file.Files;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Power;
import frc.robot.power.BatteryEstimator;

/**
 * POWER MONITOR (Oct 28): answers "was it the battery or the robot?" for every brownout and every
 * match, and keeps a history per battery so worn batteries show up before they lose a match.
 *
 * <ul>
 *   <li>Every loop: battery voltage (roboRIO), total current (PDH) and each mechanism's current
 *       (each Talon FX reports its own supply current, so no PDH channel map is needed).</li>
 *   <li>{@link BatteryEstimator}: resistance of battery + main wiring from the robot's own load.</li>
 *   <li>Rest voltage while disabled (the battery's charge before the match).</li>
 *   <li>The lowest moment of the match: volts, total amps and which mechanisms drew them. A
 *       brownout snapshots the worst moment of the second before it.</li>
 *   <li>"Test: Battery Load (shooter 2 s)" (Drive Tests tab, robot ENABLED): spins the shooter from
 *       rest for 2 s and measures R cleanly. Same robot, every battery = a fair comparison.</li>
 *   <li>/home/lvuser/shotmap/battery-history.csv: one row per match and per load test, by the
 *       "Match Battery" name typed on the Match tab. Synced to the laptop with the match data.
 *       Pit Check warns when the battery typed in had high resistance last time.</li>
 * </ul>
 * Dashboard: "Battery R (mOhm)", "Battery Rest (V)", "Robot Current (A)", "Battery Verdict",
 * "Battery Load Test". DogLog Power/*.
 */
public class PowerMonitor extends SubsystemBase {
    private static PowerDistribution pdh;

    /** The one PowerDistribution object (DogLog shares it). */
    public static synchronized PowerDistribution pdh() {
        if (pdh == null) pdh = new PowerDistribution();
        return pdh;
    }

    public static final String kHistoryFile = "/home/lvuser/shotmap/battery-history.csv";

    private record Group(String name, List<TalonFX> motors) {}
    private record Moment(double t, double volts, double amps, Map<String, Double> groups) {}

    private final List<Group> groups = new ArrayList<>();
    private final BatteryEstimator session = new BatteryEstimator(); // since boot (dashboard)
    private final BatteryEstimator match = new BatteryEstimator();   // this match
    private final ArrayDeque<Moment> lastSecond = new ArrayDeque<>();
    private final ExecutorService writer = Executors.newSingleThreadExecutor(r -> {
        Thread t = new Thread(r, "PowerMonitor writer");
        t.setDaemon(true);
        return t;
    });
    private final Map<String, Double> lastRByBattery = new HashMap<>();

    private double restVolts = Double.NaN, quietSince = -1;
    private boolean wasBrownedOut = false;
    // match scope
    private Moment matchMin = null;
    private double matchPeakAmps = 0, matchAmpSeconds = 0, matchRest = Double.NaN, lastT = -1;
    private int matchBrownouts = 0, matchBelow7 = 0;
    private boolean below7 = false;
    private final List<Map<String, Object>> brownoutSnapshots = new ArrayList<>();
    private String lastVerdict = "no match yet";
    private String loadTest = "not run";

    public PowerMonitor(CommandSwerveDrivetrain drivetrain) {
        List<TalonFX> drive = new ArrayList<>(), steer = new ArrayList<>();
        for (var m : drivetrain.getModules()) { drive.add(m.getDriveMotor()); steer.add(m.getSteerMotor()); }
        groups.add(new Group("drive", drive));
        groups.add(new Group("steer", steer));
        // Read-only handles (Phoenix 6 allows several objects per device; these never send controls).
        groups.add(new Group("shooter", List.of(new TalonFX(16), new TalonFX(17))));
        groups.add(new Group("spindexer", List.of(new TalonFX(13))));
        groups.add(new Group("uptake", List.of(new TalonFX(15))));
        groups.add(new Group("turret", List.of(new TalonFX(14))));
        groups.add(new Group("hood", List.of(new TalonFX(18))));
        groups.add(new Group("intakePivot", List.of(new TalonFX(20), new TalonFX(21))));
        groups.add(new Group("roller", List.of(new TalonFX(22))));
        groups.add(new Group("climber", List.of(new TalonFX(19))));
        loadHistory();
        SmartDashboard.putString("Battery Load Test", loadTest);
    }

    @Override
    public void periodic() {
        double t = Timer.getFPGATimestamp();
        double volts = RobotController.getBatteryVoltage();
        Map<String, Double> g = new LinkedHashMap<>();
        double motorAmps = 0;
        for (Group gr : groups) {
            double a = 0;
            for (TalonFX m : gr.motors()) a += Math.abs(m.getSupplyCurrent().getValueAsDouble());
            g.put(gr.name(), a);
            motorAmps += a;
        }
        double total = Double.NaN;
        try { total = pdh().getTotalCurrent(); } catch (Exception e) { /* no PDH answer */ }
        if (!(total >= 0) || RobotBase.isSimulation()) total = motorAmps + 3.0; // + roboRIO, radio, Limelight
        boolean enabled = DriverStation.isEnabled();

        // Rest voltage: disabled and quiet for 2 s = the battery's real charge level.
        if (!enabled && total < 6) {
            if (quietSince < 0) quietSince = t;
            if (t - quietSince > 2.0) restVolts = Double.isNaN(restVolts) ? volts : 0.9 * restVolts + 0.1 * volts;
        } else quietSince = -1;

        if (enabled) {
            session.add(t, volts, total);
            match.add(t, volts, total);
            Moment now = new Moment(t, volts, total, g);
            lastSecond.addLast(now);
            while (!lastSecond.isEmpty() && t - lastSecond.peekFirst().t() > 1.0) lastSecond.removeFirst();
            if (matchMin == null || volts < matchMin.volts()) matchMin = now;
            matchPeakAmps = Math.max(matchPeakAmps, total);
            if (lastT > 0) matchAmpSeconds += total * Math.min(0.1, t - lastT);
            boolean b7 = volts < 7.0;
            if (b7 && !below7) matchBelow7++;
            below7 = b7;
        }
        lastT = t;

        boolean bo = RobotController.isBrownedOut();
        if (bo && !wasBrownedOut) onBrownout(t);
        wasBrownedOut = bo;

        double r = session.resistance();
        SmartDashboard.putNumber("Battery R (mOhm)", Double.isNaN(r) ? -1 : Math.round(r * 10000) / 10.0);
        SmartDashboard.putNumber("Battery Rest (V)", Double.isNaN(restVolts) ? -1 : Math.round(restVolts * 100) / 100.0);
        SmartDashboard.putNumber("Robot Current (A)", Math.round(total));
        SmartDashboard.putString("Battery Verdict", lastVerdict);
        DogLog.log("Power/TotalAmps", total);
        DogLog.log("Power/BatteryVolts", volts);
        if (!Double.isNaN(session.latest())) DogLog.log("Power/ResistanceOhms", session.latest());
        for (var e : g.entrySet()) DogLog.log("Power/Amps/" + e.getKey(), e.getValue());
    }

    private void onBrownout(double t) {
        matchBrownouts++;
        Moment worst = null;
        for (Moment m : lastSecond) if (worst == null || m.volts() < worst.volts()) worst = m;
        Map<String, Object> snap = new LinkedHashMap<>();
        snap.put("t", round(t, 2));
        snap.put("matchTime", round(edu.wpi.first.wpilibj.DriverStation.getMatchTime(), 1));
        if (worst != null) {
            snap.put("lowestVolts", round(worst.volts(), 2));
            snap.put("ampsThen", Math.round(worst.amps()));
            snap.put("topDraw", topGroups(worst.groups(), 4));
        }
        snap.put("resistanceMilliOhm", Double.isNaN(match.resistance()) ? null : round(match.resistance() * 1000, 1));
        if (brownoutSnapshots.size() < 10) brownoutSnapshots.add(snap);
        String line = "BROWNOUT | " + snap;
        System.out.println(line);
        PracticeMatch.note(line);
    }

    private static Map<String, Object> topGroups(Map<String, Double> g, int n) {
        Map<String, Object> out = new LinkedHashMap<>();
        g.entrySet().stream().sorted((a, b) -> Double.compare(b.getValue(), a.getValue())).limit(n)
            .forEach(e -> out.put(e.getKey(), Math.round(e.getValue())));
        return out;
    }

    // ---------------- match scope (PracticeMatch calls these) ----------------

    public void matchStarted() {
        match.reset();
        matchMin = null;
        matchPeakAmps = matchAmpSeconds = 0;
        matchBrownouts = matchBelow7 = 0;
        brownoutSnapshots.clear();
        matchRest = restVolts;
    }

    /** The "power" section of the match file, and a battery-history row. */
    public Map<String, Object> matchFinished(String kind, String batteryLabel) {
        Map<String, Object> p = new LinkedHashMap<>();
        double r = match.resistance();
        p.put("restVoltsBefore", Double.isNaN(matchRest) ? null : round(matchRest, 2));
        p.put("resistanceMilliOhm", Double.isNaN(r) ? null : round(r * 1000, 1));
        p.put("resistanceFits", match.fits());
        p.put("openCircuitVolts", Double.isNaN(match.openCircuitVolts()) ? null : round(match.openCircuitVolts(), 2));
        p.put("minVolts", matchMin == null ? null : round(matchMin.volts(), 2));
        p.put("ampsAtMin", matchMin == null ? null : Math.round(matchMin.amps()));
        p.put("topDrawAtMin", matchMin == null ? Map.of() : topGroups(matchMin.groups(), 4));
        p.put("peakAmps", Math.round(matchPeakAmps));
        p.put("ampHours", round(matchAmpSeconds / 3600.0, 2));
        p.put("dipsBelow7V", matchBelow7);
        p.put("brownouts", matchBrownouts);
        p.put("brownoutSnapshots", new ArrayList<>(brownoutSnapshots));
        String verdict = BatteryEstimator.verdict(matchRest, r, matchMin == null ? 13 : matchMin.volts(),
            matchMin == null ? 0 : matchMin.amps(), matchBrownouts, Power.kResistanceHighOhms, Power.kRestVoltsLow);
        p.put("verdict", verdict);
        lastVerdict = (batteryLabel == null || batteryLabel.isBlank() ? "" : batteryLabel + ": ") + verdict;
        appendHistory(kind, batteryLabel, matchRest, r, matchMin == null ? Double.NaN : matchMin.volts(),
            matchMin == null ? Double.NaN : matchMin.amps(), matchBrownouts, verdict);
        return p;
    }

    // ---------------- load test ----------------

    /**
     * "Test: Battery Load (shooter 2 s)": robot ENABLED, shooter idle. Spins the shooter from rest
     * (both motors draw their supply limit while it accelerates) and fits R from the sag.
     */
    public Command loadTestCommand(ShooterSubsystem shooter) {
        BatteryEstimator est = new BatteryEstimator();
        double[] base = new double[2];
        double[] min = new double[2];
        return Commands.sequence(
            Commands.runOnce(() -> {
                est.reset();
                base[0] = RobotController.getBatteryVoltage();
                base[1] = SmartDashboard.getNumber("Robot Current (A)", 0);
                min[0] = 99; min[1] = 0;
                loadTest = "running...";
            }),
            Commands.run(() -> {
                shooter.runAtRPM(Constants_kTestRpm);
                double v = RobotController.getBatteryVoltage(), a = SmartDashboard.getNumber("Robot Current (A)", 0);
                est.add(Timer.getFPGATimestamp(), v, a);
                if (v < min[0]) { min[0] = v; min[1] = a; }
            }, shooter).withTimeout(2.0),
            Commands.runOnce(() -> {
                shooter.stop();
                double r = est.resistance();
                if (Double.isNaN(r) && min[1] - base[1] > 30) r = (base[0] - min[0]) / (min[1] - base[1]);
                String label = SmartDashboard.getString("Match Battery", "");
                loadTest = Double.isNaN(r)
                    ? String.format(Locale.US, "no reading (current only rose %.0f A): is the robot enabled?", min[1] - base[1])
                    : String.format(Locale.US, "%s R = %.1f mOhm (battery + wiring) | %.2f V idle -> %.2f V at %.0f A | %s",
                        label.isBlank() ? "battery ?" : label, r * 1000, base[0], min[0], min[1],
                        r > Power.kResistanceHighOhms ? "HIGH: compare with another battery" : "normal");
                System.out.println("BATTERY TEST | " + loadTest);
                if (!Double.isNaN(r)) appendHistory("load_test", label, base[0], r, min[0], min[1], 0, loadTest);
                SmartDashboard.putString("Battery Load Test", loadTest);
            })).finallyDo(shooter::stop).withName("Test: Battery Load");
    }

    private static final double Constants_kTestRpm = 4500;

    // ---------------- per-battery history ----------------

    private void appendHistory(String kind, String label, double rest, double r, double minV, double ampsAtMin,
                               int brownouts, String verdict) {
        String l = label == null ? "" : label.trim();
        if (!l.isEmpty() && !Double.isNaN(r)) lastRByBattery.put(l.toLowerCase(Locale.US), r);
        String row = String.join(",",
            LocalDateTime.now().format(DateTimeFormatter.ISO_LOCAL_DATE_TIME), csv(kind), csv(l),
            fmt(rest, 2), fmt(r * 1000, 1), fmt(minV, 2), fmt(ampsAtMin, 0), String.valueOf(brownouts), csv(verdict)) + "\n";
        writer.execute(() -> {
            try {
                File f = new File(kHistoryFile);
                f.getParentFile().mkdirs();
                boolean header = !f.exists() || f.length() == 0;
                try (FileWriter w = new FileWriter(f, true)) {
                    if (header) w.write("time,kind,battery,rest_v,resistance_mohm,min_v,amps_at_min,brownouts,verdict\n");
                    w.write(row);
                }
            } catch (Exception e) {
                System.out.println("POWER | could not write battery history: " + e.getMessage());
            }
        });
    }

    private void loadHistory() {
        try {
            File f = new File(kHistoryFile);
            if (!f.exists()) return;
            for (String line : Files.readAllLines(f.toPath())) {
                String[] c = line.split(",", -1);
                if (c.length < 5 || c[0].equals("time") || c[2].isBlank() || c[4].isBlank()) continue;
                try { lastRByBattery.put(c[2].replace("\"", "").trim().toLowerCase(Locale.US), Double.parseDouble(c[4]) / 1000.0); }
                catch (NumberFormatException ignored) { }
            }
        } catch (Exception e) { /* first boot */ }
    }

    /** Pit Check: a warning when the battery typed in had high resistance last time, else null. */
    public String batteryWarning(String label) {
        if (label == null || label.isBlank()) return null;
        Double r = lastRByBattery.get(label.trim().toLowerCase(Locale.US));
        if (r == null || r <= Power.kResistanceHighOhms) return null;
        return String.format(Locale.US, "battery %s read %.0f mOhm last time (high): swap it or load-test it", label.trim(), r * 1000);
    }

    private static String fmt(double v, int d) { return Double.isNaN(v) ? "" : String.format(Locale.US, "%." + d + "f", v); }
    private static double round(double v, int d) { double f = Math.pow(10, d); return Math.round(v * f) / f; }
    private static String csv(String v) {
        if (v == null) return "";
        return v.contains(",") || v.contains("\"") ? "\"" + v.replace("\"", "\"\"") + "\"" : v;
    }
}
