package frc.robot.aiming;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Picks WHAT to shoot at from where the robot is, the way elite 2026 teams did it (6328, 581, 1678
 * public code): on our side of our hub = SCORE into our hub; past our hub (neutral zone, or deeper)
 * = PASS to a landing point in a corner of our own alliance zone, near our wall, on the robot's
 * side of the field. If the straight line to that corner crosses either hub (plus padding), it
 * uses a backup point closer to the side wall (581's approach).
 *
 * <p>Coordinates are WPILib blue-origin meters. Everything is described from OUR alliance's point
 * of view ("depth" = distance from our alliance wall) and mirrored for red.
 *
 * <p>Pure math, no hardware: tested in TargetPickerTest.
 */
public final class TargetPicker {
    public enum Kind { SCORE, PASS }

    /** Operator choice: AUTO picks by field position. */
    public enum Mode { AUTO, SCORE, PASS }

    /** Field and target numbers (meters). */
    public record Field(
        double length, double width,
        double hubDepth,            // our hub center, distance from our wall
        double hubY,
        double hubHalfSize,         // half the hub's footprint (square)
        double scoreLinePastHub,    // AUTO: score while within this far past our hub center
        double hysteresis,          // so it doesn't flicker at the line
        double passDepth, double passSide,          // primary pass point: from our wall / from the side wall
        double backupDepth, double backupSide,      // backup pass point
        double obstaclePad) {}

    public record Target(Kind kind, Translation2d point, String name) {}

    private final Field f;
    private Kind last = Kind.SCORE;

    public TargetPicker(Field f) { this.f = f; }

    /** Distance from OUR alliance wall. */
    public double depth(Translation2d p, boolean red) {
        return red ? f.length() - p.getX() : p.getX();
    }

    /** A point given as (depth from our wall, y) in field coordinates. */
    public Translation2d fromOurWall(double depth, double y, boolean red) {
        return new Translation2d(red ? f.length() - depth : depth, y);
    }

    public Translation2d ourHub(boolean red) { return fromOurWall(f.hubDepth(), f.hubY(), red); }
    public Translation2d theirHub(boolean red) { return fromOurWall(f.length() - f.hubDepth(), f.hubY(), red); }

    /**
     * @param launcher where the ball leaves (turret pivot), field coordinates
     * @param red      our alliance is red
     * @param mode     AUTO / forced SCORE / forced PASS
     */
    public Target pick(Translation2d launcher, boolean red, Mode mode) {
        Kind kind;
        if (mode == Mode.SCORE) kind = Kind.SCORE;
        else if (mode == Mode.PASS) kind = Kind.PASS;
        else {
            double line = f.hubDepth() + f.scoreLinePastHub();
            double d = depth(launcher, red);
            if (last == Kind.SCORE) kind = d > line + f.hysteresis() ? Kind.PASS : Kind.SCORE;
            else kind = d < line - f.hysteresis() ? Kind.SCORE : Kind.PASS;
        }
        last = kind;
        if (kind == Kind.SCORE) return new Target(Kind.SCORE, ourHub(red), "HUB");
        return passTarget(launcher, red);
    }

    /** The pass point on the launcher's side of the field, or its backup if a hub is in the way. */
    public Target passTarget(Translation2d launcher, boolean red) {
        boolean lowSide = launcher.getY() < f.width() / 2;
        double y = lowSide ? f.passSide() : f.width() - f.passSide();
        Translation2d primary = fromOurWall(f.passDepth(), y, red);
        String side = lowSide ? "low-y" : "high-y";
        if (!blocked(launcher, primary, red)) return new Target(Kind.PASS, primary, "PASS " + side + " corner");
        double by = lowSide ? f.backupSide() : f.width() - f.backupSide();
        Translation2d backup = fromOurWall(f.backupDepth(), by, red);
        return new Target(Kind.PASS, backup, "PASS " + side + " backup");
    }

    /** True if the segment a-b passes over either hub's footprint (plus padding). */
    public boolean blocked(Translation2d a, Translation2d b, boolean red) {
        double half = f.hubHalfSize() + f.obstaclePad();
        return segmentHitsBox(a, b, ourHub(red), half) || segmentHitsBox(a, b, theirHub(red), half);
    }

    /** Segment vs axis-aligned square (Liang-Barsky clip). */
    static boolean segmentHitsBox(Translation2d a, Translation2d b, Translation2d c, double half) {
        double x0 = a.getX(), y0 = a.getY(), dx = b.getX() - x0, dy = b.getY() - y0;
        double t0 = 0, t1 = 1;
        double[] p = {-dx, dx, -dy, dy};
        double[] q = {x0 - (c.getX() - half), (c.getX() + half) - x0, y0 - (c.getY() - half), (c.getY() + half) - y0};
        for (int i = 0; i < 4; i++) {
            if (p[i] == 0) {
                if (q[i] < 0) return false;
            } else {
                double t = q[i] / p[i];
                if (p[i] < 0) t0 = Math.max(t0, t); else t1 = Math.min(t1, t);
                if (t0 > t1) return false;
            }
        }
        return true;
    }
}
