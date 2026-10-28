package frc.robot.practice;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * MATCH TELEMETRY (Oct 28): one summary per match (practice or real), built from samples the robot
 * takes every loop, so a whole match fits in one small JSON file that people, the FRC app and Claude
 * can read and compare. The full every-signal log stays in the .wpilog on the roboRIO's USB stick
 * (AdvantageScope); this is the scorecard on top of it.
 *
 * <p>Pure (no WPILib calls): {@link frc.robot.subsystems.PracticeMatch} feeds it, tests drive it.
 * Schema version {@link #kSchema}; field names are the contract with the FRC app import.
 */
public final class MatchTelemetry {
    public static final int kSchema = 1;

    /** One loop's worth of readings. */
    public record Sample(double t, double matchClock, String phase, double battery, boolean brownedOut,
                         double driveSupplyAmps, double speed, boolean aimOn, boolean ready, boolean rtHeld,
                         boolean firing, String notReadyReason, boolean vacuum, boolean dump, boolean defense,
                         boolean poseTrusted, double secondsSinceVisionFix, boolean climbing, boolean hanging,
                         boolean powerSag, int canBusOff, double canUtil) {}

    private static final class PhaseStats {
        double seconds, minBattery = 99, sumBattery; int samples, shots;
    }

    private final Map<String, Object> meta = new LinkedHashMap<>();
    private final Map<String, PhaseStats> phases = new LinkedHashMap<>();
    private final Map<String, Double> notReady = new LinkedHashMap<>();
    private final List<Map<String, Object>> events = new ArrayList<>();
    private Sample last = null;
    private double battStart = -1, battEnd = -1, battMin = 99, battSum = 0, below10 = 0, below8 = 0;
    private String battMinWhere = "";
    private int samples = 0, brownouts = 0, canBusOffStart = -1, canBusOffEnd = 0;
    private double canUtilMax = 0, distance = 0, topSpeed = 0, ampsSum = 0, ampsPeak = 0;
    private double aimOn = 0, pose = 0, maxNoFix = 0, rtHeld = 0, readyWhileRt = 0, firing = 0;
    private double vacuum = 0, dump = 0, defense = 0, sag = 0, total = 0;
    private int autoBalls, openBalls, closedBalls, passBalls, launches;
    private double launchErrAbsSum = 0;
    private double liftAt = -1, hangAt = -1;
    private boolean hangingAtEnd = false;

    public MatchTelemetry(Map<String, Object> meta) {
        this.meta.putAll(meta);
    }

    public void sample(Sample s) {
        double dt = last == null ? 0 : Math.max(0, Math.min(0.1, s.t() - last.t()));
        samples++;
        total += dt;
        if (battStart < 0 && s.battery() > 0) battStart = s.battery();
        battEnd = s.battery();
        battSum += s.battery();
        if (s.battery() < battMin) { battMin = s.battery(); battMinWhere = s.phase() + " " + clock(s.matchClock()); }
        if (s.battery() < 10) below10 += dt;
        if (s.battery() < 8) below8 += dt;
        if (s.brownedOut() && (last == null || !last.brownedOut())) { brownouts++; event(s, "BROWNOUT"); }
        PhaseStats p = phases.computeIfAbsent(s.phase(), k -> new PhaseStats());
        p.seconds += dt; p.samples++; p.sumBattery += s.battery(); p.minBattery = Math.min(p.minBattery, s.battery());
        distance += s.speed() * dt;
        topSpeed = Math.max(topSpeed, s.speed());
        ampsSum += s.driveSupplyAmps() * dt;
        ampsPeak = Math.max(ampsPeak, s.driveSupplyAmps());
        if (s.aimOn()) aimOn += dt;
        if (s.poseTrusted()) pose += dt;
        maxNoFix = Math.max(maxNoFix, s.secondsSinceVisionFix());
        if (s.rtHeld()) {
            rtHeld += dt;
            if (s.ready()) readyWhileRt += dt;
            else if (s.notReadyReason() != null && !s.notReadyReason().isBlank() && !s.notReadyReason().equals("-")
                    && dt > 0) notReady.merge(reason(s.notReadyReason()), dt, Double::sum);
        }
        if (s.firing()) firing += dt;
        if (s.vacuum()) vacuum += dt;
        if (s.dump()) dump += dt;
        if (s.defense()) defense += dt;
        if (s.powerSag()) sag += dt;
        if (canBusOffStart < 0) canBusOffStart = s.canBusOff();
        canBusOffEnd = s.canBusOff();
        canUtilMax = Math.max(canUtilMax, s.canUtil());
        if (liftAt < 0 && s.climbing()) liftAt = s.t();
        if (hangAt < 0 && s.hanging()) hangAt = s.t();
        hangingAtEnd = s.hanging();
        if (last != null) {
            if (!s.phase().equals(last.phase())) event(s, "phase " + s.phase());
            edge(s, last.aimOn(), s.aimOn(), "aim ON", "aim OFF");
            edge(s, last.vacuum(), s.vacuum(), "vacuum ON", "vacuum OFF");
            edge(s, last.dump(), s.dump(), "dump ON", "dump OFF");
            edge(s, last.defense(), s.defense(), "defense ON", "defense OFF");
            edge(s, last.climbing(), s.climbing(), "climber up", "climber down");
            edge(s, last.hanging(), s.hanging(), "hanging", "not hanging");
            edge(s, last.poseTrusted(), s.poseTrusted(), "position known", "position LOST");
            edge(s, last.powerSag(), s.powerSag(), "battery SAG (power budget)", "sag over");
            if (s.canBusOff() > last.canBusOff()) event(s, "CAN BUS OFF");
        }
        last = s;
    }

    private void edge(Sample s, boolean before, boolean now, String on, String off) {
        if (before != now) event(s, now ? on : off);
    }

    /** "WAITING: spinning up (3200 of 3600)" -> "WAITING: spinning up". */
    static String reason(String r) {
        String x = r.replaceAll("\\(.*?\\)", "").replaceAll("[0-9.]+", "#").trim();
        return x.length() > 60 ? x.substring(0, 60) : x;
    }

    /** A ball left: category auto / open / closed / pass. */
    public void shot(String phase, String category) {
        PhaseStats p = phases.computeIfAbsent(phase, k -> new PhaseStats());
        p.shots++;
        switch (category) {
            case "auto" -> autoBalls++;
            case "pass" -> passBalls++;
            case "closed" -> closedBalls++;
            default -> openBalls++;
        }
    }

    /** A finished ball's launch speed vs the target (RPM). */
    public void launch(double launchRpm, double targetRpm) {
        if (targetRpm <= 0) return;
        launches++;
        launchErrAbsSum += Math.abs(launchRpm - targetRpm);
    }

    /** Anything worth a line on the timeline (command interrupts, auto end...). */
    public void event(double t, double matchClock, String phase, String text) {
        if (events.size() >= 400) return;
        Map<String, Object> e = new LinkedHashMap<>();
        e.put("t", round(t, 2));
        e.put("clock", clock(matchClock));
        e.put("phase", phase);
        e.put("text", text);
        events.add(e);
    }

    private void event(Sample s, String text) { event(s.t(), s.matchClock(), s.phase(), text); }

    public int autoBalls() { return autoBalls; }
    public int openBalls() { return openBalls; }
    public int closedBalls() { return closedBalls; }
    public int passBalls() { return passBalls; }
    public double liftAt() { return liftAt; }
    public double hangAt() { return hangAt; }
    public double batteryMin() { return battMin > 90 ? -1 : battMin; }

    static String clock(double s) {
        if (s < 0) return "-";
        int x = (int) Math.ceil(s);
        return x / 60 + ":" + String.format("%02d", x % 60);
    }

    static double round(double v, int d) {
        double f = Math.pow(10, d);
        return Math.round(v * f) / f;
    }

    private static double pct(double part, double whole) { return whole <= 0 ? 0 : round(100 * part / whole, 1); }

    /** The whole match as nested maps (written as JSON). */
    public Map<String, Object> summary() {
        Map<String, Object> out = new LinkedHashMap<>();
        out.put("schema", kSchema);
        out.putAll(meta);
        out.put("durationSeconds", round(total, 1));

        Map<String, Object> scoring = new LinkedHashMap<>();
        scoring.put("autoBalls", autoBalls);
        scoring.put("ballsHubOpen", openBalls);
        scoring.put("ballsHubClosed", closedBalls);
        scoring.put("passes", passBalls);
        scoring.put("ballsTotal", autoBalls + openBalls + closedBalls + passBalls);
        scoring.put("rtHeldSeconds", round(rtHeld, 1));
        scoring.put("readyWhileRtHeldPct", pct(readyWhileRt, rtHeld));
        scoring.put("firingSeconds", round(firing, 1));
        scoring.put("ballsPerFiringSecond", firing > 0 ? round((openBalls + closedBalls + passBalls + autoBalls) / firing, 2) : 0);
        scoring.put("launchRpmErrorMean", launches > 0 ? round(launchErrAbsSum / launches, 0) : null);
        Map<String, Object> nr = new LinkedHashMap<>();
        notReady.entrySet().stream().sorted((a, b) -> Double.compare(b.getValue(), a.getValue())).limit(8)
            .forEach(e -> nr.put(e.getKey(), round(e.getValue(), 1)));
        scoring.put("notReadySecondsByReason", nr);
        out.put("scoring", scoring);

        Map<String, Object> batt = new LinkedHashMap<>();
        batt.put("start", round(battStart, 2));
        batt.put("end", round(battEnd, 2));
        batt.put("min", round(batteryMin(), 2));
        batt.put("minWhere", battMinWhere);
        batt.put("avg", samples > 0 ? round(battSum / samples, 2) : null);
        batt.put("secondsBelow10V", round(below10, 1));
        batt.put("secondsBelow8V", round(below8, 1));
        batt.put("brownouts", brownouts);
        batt.put("powerSagSeconds", round(sag, 1));
        out.put("battery", batt);

        Map<String, Object> drive = new LinkedHashMap<>();
        drive.put("distanceMeters", round(distance, 1));
        drive.put("topSpeedMps", round(topSpeed, 2));
        drive.put("driveSupplyAmpsAvg", total > 0 ? round(ampsSum / total, 1) : 0);
        drive.put("driveSupplyAmpsPeak", round(ampsPeak, 1));
        drive.put("defenseSeconds", round(defense, 1));
        drive.put("vacuumSeconds", round(vacuum, 1));
        drive.put("dumpSeconds", round(dump, 1));
        out.put("drive", drive);

        Map<String, Object> aim = new LinkedHashMap<>();
        aim.put("aimOnPct", pct(aimOn, total));
        aim.put("positionKnownPct", pct(pose, total));
        aim.put("longestSecondsWithoutVisionFix", round(Math.min(maxNoFix, 999), 1));
        out.put("aim", aim);

        Map<String, Object> climb = new LinkedHashMap<>();
        climb.put("liftAtSeconds", liftAt < 0 ? null : round(liftAt, 1));
        climb.put("hangAtSeconds", hangAt < 0 ? null : round(hangAt, 1));
        climb.put("hangingAtEnd", hangingAtEnd);
        out.put("climb", climb);

        Map<String, Object> can = new LinkedHashMap<>();
        can.put("busOffEvents", Math.max(0, canBusOffEnd - Math.max(0, canBusOffStart)));
        can.put("utilizationMaxPct", round(canUtilMax * 100, 1));
        out.put("can", can);

        List<Map<String, Object>> ph = new ArrayList<>();
        phases.forEach((name, p) -> {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("phase", name);
            m.put("seconds", round(p.seconds, 1));
            m.put("shots", p.shots);
            m.put("batteryMin", p.samples > 0 ? round(p.minBattery, 2) : null);
            m.put("batteryAvg", p.samples > 0 ? round(p.sumBattery / p.samples, 2) : null);
            ph.add(m);
        });
        out.put("phases", ph);
        out.put("events", events);
        return out;
    }
}
