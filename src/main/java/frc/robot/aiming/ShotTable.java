package frc.robot.aiming;

import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Distance to the hub (meters, from the turret pivot) -> shooter RPM, hood position (rotations)
 * and ball time of flight (seconds). Between points it interpolates in a straight line; outside
 * the measured range it uses the nearest end point.
 *
 * <p>Points come from two places: the starting guesses in {@link #defaults()}, and points recorded
 * on the robot during a tuning session (saved on the roboRIO, see ShotTuning). A recorded point
 * replaces any default within 0.25 m of it.
 */
public final class ShotTable {
    /** One measured shot setting. */
    public record Point(double distance, double rpm, double hood, double tof) {}

    private final TreeMap<Double, Point> points = new TreeMap<>();
    private InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();

    /** Points closer together than this (meters) count as the same distance. */
    public static final double kSameDistance = 0.25;

    /**
     * MEASURED Oct 28 evening (rebuilt hood, shop field, Plot Spot pose, Shot Tuning Mode), read
     * from the Driver Station log "SHOT TABLE CODE" line. Only real shots: the old guesses are gone,
     * because a guess left between two real points (2.5 and 3.5 m) bent the curve.
     * Still to record: Midshot (~2.4 m) and Trench (~3.7 m) backup spots, which fill the two
     * biggest gaps. 1.69 m was recorded at Tune Hood 0.0, which the hood clamps to its 0.2
     * bottom, so 0.20 is what really flew. 5.33 m needs 4300 RPM, about all the wheel can hold on
     * an 11 V battery: long shots want a fresh battery. Time of flight is the 0.9 s placeholder
     * everywhere (only used to lead shots while moving; film shots to measure it).
     */
    public static ShotTable defaults() {
        ShotTable t = new ShotTable();
        t.put(new Point(1.69, 2500, 0.20, 0.90));
        t.put(new Point(3.13, 2900, 3.00, 0.90));
        t.put(new Point(4.73, 3500, 4.50, 0.90));
        t.put(new Point(5.33, 4300, 5.50, 0.90));
        return t;
    }

    /**
     * SHOOT WHILE CLIMBING (elevator up): starts as a copy of the SCORE table. Hanging lifts the
     * shooter, so record real points from the tower ("Record Shot Point" while climbing records
     * here) at both climb spots (left and right end of the rung).
     */
    public static ShotTable climbDefaults() {
        return defaults();
    }

    /**
     * !!! PASSING STARTING GUESSES, NOT MEASURED: distance from the turret to the landing point
     * (meters) -> RPM, hood, flight time. Passes are lower, longer lobs onto the floor. Hood high,
     * RPM rising with distance (the elite teams' pattern: 6328 fixed its hood at max and only
     * changed speed). Record real ones with "Shot Tuning Mode" while passing ("Shot Target" PASS).
     */
    public static ShotTable passDefaults() {
        ShotTable t = new ShotTable();
        // 3.5 to 13 m: driving at 4 m/s shifts a 7.7 m pass to ~4 m (toward home) or ~12.5 m (away).
        // Oct 27: hood at the TOP for every pass (raising the hood flattens the shot: further, not
        // higher; 6328 fixed its hood at max for passes and only changed RPM).
        t.put(new Point(3.5, 3000, 7.75, 0.95));
        t.put(new Point(6.0, 3500, 7.75, 1.20));
        t.put(new Point(8.0, 3850, 7.75, 1.40));
        t.put(new Point(10.0, 4150, 7.75, 1.60));
        t.put(new Point(13.0, 4500, 7.75, 1.90));
        return t;
    }

    /** Adds a point, replacing any point within kSameDistance of it. */
    public void put(Point p) {
        points.subMap(p.distance() - kSameDistance, false, p.distance() + kSameDistance, false).clear();
        points.put(p.distance(), p);
        rebuild();
    }

    private void rebuild() {
        rpmMap = new InterpolatingDoubleTreeMap();
        hoodMap = new InterpolatingDoubleTreeMap();
        tofMap = new InterpolatingDoubleTreeMap();
        for (Point p : points.values()) {
            rpmMap.put(p.distance(), p.rpm());
            hoodMap.put(p.distance(), p.hood());
            tofMap.put(p.distance(), p.tof());
        }
    }

    public double rpm(double distance) { return rpmMap.get(distance); }
    public double hood(double distance) { return hoodMap.get(distance); }
    public double timeOfFlight(double distance) { return tofMap.get(distance); }

    public Map<Double, Point> points() { return java.util.Collections.unmodifiableMap(points); }

    /** "d:rpm:hood:tof;..." for saving on the roboRIO. */
    public static String encode(Iterable<Point> pts) {
        StringBuilder sb = new StringBuilder();
        for (Point p : pts) {
            if (sb.length() > 0) sb.append(';');
            sb.append(String.format(java.util.Locale.US, "%.2f:%.0f:%.3f:%.2f", p.distance(), p.rpm(), p.hood(), p.tof()));
        }
        return sb.toString();
    }

    /** Reads what encode() wrote. Bad entries are skipped. */
    public static java.util.List<Point> decode(String s) {
        java.util.List<Point> out = new java.util.ArrayList<>();
        if (s == null || s.isBlank()) return out;
        for (String part : s.split(";")) {
            String[] f = part.split(":");
            if (f.length != 4) continue;
            try {
                out.add(new Point(Double.parseDouble(f[0]), Double.parseDouble(f[1]),
                    Double.parseDouble(f[2]), Double.parseDouble(f[3])));
            } catch (NumberFormatException e) {
                // skip
            }
        }
        return out;
    }

    /** Readable table for the dashboard. */
    public String describe() {
        StringBuilder sb = new StringBuilder();
        for (Point p : points.values()) {
            sb.append(String.format(java.util.Locale.US, "%.1fm: %.0f rpm, hood %.2f | ", p.distance(), p.rpm(), p.hood()));
        }
        return sb.toString();
    }
}
