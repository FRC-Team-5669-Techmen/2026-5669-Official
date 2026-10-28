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

    private static Boolean pdhOk = null;

    /**
     * Is a power distribution module answering on CAN? Checked ONCE (the first read), because
     * every read of a missing one prints "CAN: Message not found" and slows the loop (seen on the
     * robot Oct 28). Without one, total current = the Talons' sum + 3 A.
     */
    public static synchronized boolean pdhPresent() {
        if (pdhOk == null) {
            if (RobotBase.isSimulation()) { pdhOk = false; return false; }
            double v = 0;
            try { v = pdh().getVoltage(); } catch (Exception e) { v = 0; }
            pdhOk = v > 3 && v < 16;
            System.out.println("POWER | power distribution " + (pdhOk ? "found (" + v + " V)" : "NOT answering on CAN: using the Talons' current"));
        }
        return pdhOk;
    }

    private final java.util.Set<TalonFX> connected = new java.util.HashSet<>();
    private final Map<TalonFX, com.ctre.phoenix6.StatusSignal<edu.wpi.first.units.measure.Current>> currentSignals = new HashMap<>();
    private com.ctre.phoenix6.BaseStatusSignal[] signals = new com.ctre.phoenix6.BaseStatusSignal[0];
    private double lastConnCheck = -10;

    private void refreshConnected(double t) {
        lastConnCheck = t;
        java.util.Set<TalonFX> now = new java.util.HashSet<>();
        for (Group gr : groups) for (TalonFX m : gr.motors()) {
            currentSignals.computeIfAbsent(m, k -> k.getSupplyCurrent(false));
            if (m.isConnected(0.5)) now.add(m);
        }
        if (!now.equals(connected)) {
            connected.clear();
            connected.addAll(now);
            List<com.ctre.phoenix6.BaseStatusSignal> l = new ArrayList<>();
            for (TalonFX m : connected) l.add(currentSignals.get(m));
            signals = l.toArray(new com.ctre.phoenix6.BaseStatusSignal[0]);
        }
    }

    public static final String kHistoryFile = "/home/lvuser/shotmap/battery-history.csv";

    private record Group(String name, List<TalonFX> motors) {}
    private record Moment(double t, double volts, double rioVolts, double amps, Map<String, Double> groups,
                          Map<String, Double> groupVolts) {}

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
    private double matchRioMin = 99, matchMotorAmpsAtPeak = 0;
    private String lastMatchBattery = null;
    private volatile List<Map<String, Object>> lastStickyFaults = List.of();
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
        // Research Oct 28: fit on the PDH's own voltage (battery side of the robot), not the
        // roboRIO's input, which sits downstream of the PDH and its own wires. Both are logged: a
        // growing gap between them = a bad roboRIO feed.
        double rioVolts = RobotController.getBatteryVoltage();
        double volts = rioVolts;
        if (RobotBase.isReal() && pdhPresent()) {
            try { double pv = pdh().getVoltage(); if (pv > 3 && pv < 16) volts = pv; } catch (Exception e) { /* no PDH */ }
        }
        // Oct 28 (on the robot): reading every Talon's SupplyVoltage at 50 Hz printed "stale"
        // warnings (that signal updates slowly) and overran the loop, so only supply CURRENT is
        // read, all signals refreshed in ONE call, and devices that are off the bus (Talon 22)
        // are skipped (checked every 2 s) so they don't print "CAN frame not received".
        if (t - lastConnCheck > 2.0) refreshConnected(t);
        if (signals.length > 0) com.ctre.phoenix6.BaseStatusSignal.refreshAll(signals);
        Map<String, Double> g = new LinkedHashMap<>();
        Map<String, Double> gv = new LinkedHashMap<>();
        double motorAmps = 0;
        for (Group gr : groups) {
            double a = 0;
            for (TalonFX m : gr.motors()) {
                var sig = currentSignals.get(m);
                if (sig != null && connected.contains(m)) a += Math.abs(sig.getValueAsDouble());
            }
            g.put(gr.name(), a);
            gv.put(gr.name(), Double.NaN);
            motorAmps += a;
        }
        double total = Double.NaN;
        if (pdhPresent()) { try { total = pdh().getTotalCurrent(); } catch (Exception e) { /* no PDH answer */ } }
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
            Moment now = new Moment(t, volts, rioVolts, total, g, gv);
            matchRioMin = Math.min(matchRioMin, rioVolts);
            matchMotorAmpsAtPeak = total >= matchPeakAmps ? motorAmps : matchMotorAmpsAtPeak;
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
        DogLog.log("Power/RioVolts", rioVolts);
        DogLog.log("Power/TalonAmps", motorAmps);
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
        matchRioMin = 99;
        matchMotorAmpsAtPeak = 0;
        writer.execute(this::clearStickies); // so only this match's faults count
    }

    private List<TalonFX> allTalons() {
        List<TalonFX> all = new ArrayList<>();
        for (Group gr : groups) all.addAll(gr.motors());
        return all;
    }

    private void clearStickies() {
        for (TalonFX m : allTalons()) {
            if (!connected.contains(m)) continue;
            try { m.clearStickyFaults(0.05); } catch (Exception e) { /* off the bus */ }
        }
    }

    /**
     * Talons that saw their supply sag during the match (sticky Undervoltage / BridgeBrownout,
     * Research Oct 28): the robot's 50 Hz loop can miss a short dip, the motor controllers don't.
     * One motor alone = its own branch (breaker, WAGO terminal, wire); many = the main path or battery.
     */
    private List<Map<String, Object>> readStickies() {
        List<Map<String, Object>> out = new ArrayList<>();
        for (Group gr : groups) {
            for (TalonFX m : gr.motors()) {
                if (!connected.contains(m)) continue;
                List<String> f = new ArrayList<>();
                try {
                    if (Boolean.TRUE.equals(m.getStickyFault_Undervoltage().getValue())) f.add("undervoltage");
                    if (Boolean.TRUE.equals(m.getStickyFault_BridgeBrownout().getValue())) f.add("bridge brownout");
                } catch (Exception e) { /* off the bus */ }
                if (!f.isEmpty()) out.add(Map.of("device", gr.name() + " " + m.getDeviceID(), "faults", f));
            }
        }
        return out;
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
        p.put("pdhMinVolts", matchMin == null ? null : round(matchMin.volts(), 2));
        p.put("rioMinVolts", matchRioMin > 90 ? null : round(matchRioMin, 2));
        List<Map<String, Object>> sticky = readStickies();
        lastStickyFaults = sticky;
        p.put("stickyFaults", sticky);
        String lbl = batteryLabel == null ? "" : batteryLabel.trim();
        p.put("sameBatteryAsLastMatch", !lbl.isEmpty() && lbl.equalsIgnoreCase(lastMatchBattery));
        if (!lbl.isEmpty() && !"teleop_session".equals(kind) && !"auto_test".equals(kind)) lastMatchBattery = lbl;
        p.put("ampsAtMin", matchMin == null ? null : Math.round(matchMin.amps()));
        p.put("topDrawAtMin", matchMin == null ? Map.of() : topGroups(matchMin.groups(), 4));
        p.put("peakAmps", Math.round(matchPeakAmps));
        p.put("ampHours", round(matchAmpSeconds / 3600.0, 2));
        p.put("dipsBelow7V", matchBelow7);
        p.put("brownouts", matchBrownouts);
        p.put("brownoutSnapshots", new ArrayList<>(brownoutSnapshots));
        String verdict = BatteryEstimator.verdict(matchRest, r, matchMin == null ? 13 : matchMin.volts(),
            matchMin == null ? 0 : matchMin.amps(), matchBrownouts, Power.kResistanceHighOhms, Power.kRestVoltsLow);
        if (!sticky.isEmpty() && sticky.size() <= 2 && matchBrownouts == 0)
            verdict = (verdict.equals("OK") ? "" : verdict + " | ") + "BRANCH: only " + sticky.get(0).get("device")
                + " saw its supply sag: check that motor's breaker, PDH terminal and wire";
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
        if (DriverStation.isFMSAttached() && label.trim().equalsIgnoreCase(lastMatchBattery))
            return "battery " + label.trim() + " was in the LAST match too: swap it (or retype Match Battery)";
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
