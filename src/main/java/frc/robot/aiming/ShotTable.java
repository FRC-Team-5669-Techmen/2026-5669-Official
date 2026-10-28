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
     * !!! STARTING GUESSES, NOT MEASURED. Replace by recording real shots (CLAUDE.md "Shot table").
     * RPM: 3100 could not reach the hub from the usual spot, 5400 is the fast preset. Hood: the
     * scaled numbers from last season's map. Time of flight: rough projectile estimates, only used
     * to aim ahead while driving.
     */
    public static ShotTable defaults() {
        ShotTable t = new ShotTable();
        t.put(new Point(1.5, 3400, 0.36, 0.55));
        t.put(new Point(2.5, 3900, 0.70, 0.70));
        t.put(new Point(3.5, 4400, 1.00, 0.85));
        t.put(new Point(4.5, 4900, 1.38, 1.00));
        t.put(new Point(5.5, 5300, 1.70, 1.15));
        t.put(new Point(6.5, 5600, 1.85, 1.30));
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
        t.put(new Point(3.5, 3000, 3.80, 0.95));
        t.put(new Point(6.0, 3500, 3.80, 1.20));
        t.put(new Point(8.0, 3850, 3.80, 1.40));
        t.put(new Point(10.0, 4150, 3.80, 1.60));
        t.put(new Point(13.0, 4500, 3.80, 1.90));
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
