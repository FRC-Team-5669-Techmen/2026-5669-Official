package frc.robot.aiming;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Picks WHAT to shoot at from where the robot is, the way elite 2026 teams did it (6328, 581, 1678
 * public code), within the rules: G407 lets a robot launch into its hub only while its BUMPERS are
 * at least partly inside its ALLIANCE ZONE (MAJOR FOUL otherwise). So: robot center close enough
 * to our wall that the bumpers must be in the zone = SCORE into our hub; further out (neutral
 * zone, or deeper) = PASS to a landing point in a corner of our alliance zone, near our wall, on
 * the robot's side of the field. If the straight line to that corner crosses either hub (plus
 * padding: a pass that drops into our hub from outside the zone is a G407 foul too), it uses a
 * backup point closer to the side wall (581's approach).
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
        double scoreMaxDepth,       // robot CENTER this close to our wall or closer = legal to score (G407)
        double hysteresis,          // so it doesn't flicker at the line
        double passDepth, double passSide,          // primary pass point: from our wall / from the side wall
        double backupDepth, double backupSide,      // backup pass point
        double obstaclePad,
        double maxPassRange,                        // longest pass the shooter can make (m)
        double relayDepth, double relaySide) {      // SHUTTLE relay point in the neutral zone, our half
        /** Without shuttle settings: corners only, any range. */
        public Field(double length, double width, double hubDepth, double hubY, double hubHalfSize,
                     double scoreMaxDepth, double hysteresis, double passDepth, double passSide,
                     double backupDepth, double backupSide, double obstaclePad) {
            this(length, width, hubDepth, hubY, hubHalfSize, scoreMaxDepth, hysteresis, passDepth, passSide,
                backupDepth, backupSide, obstaclePad, Double.POSITIVE_INFINITY, 6.0, 1.65);
        }
    }

    public record Target(Kind kind, Translation2d point, String name, boolean laneClear) {
        public Target(Kind kind, Translation2d point, String name) { this(kind, point, name, true); }
    }

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

    /** G407: the robot's bumpers are surely (partly) inside our alliance zone. */
    public boolean legalToScore(Translation2d robotCenter, boolean red) {
        return depth(robotCenter, red) <= f.scoreMaxDepth();
    }

    /**
     * @param robotCenter robot center (decides score vs pass: the zone rule is about the bumpers)
     * @param launcher    where the ball leaves (turret pivot): picks the pass corner and checks the lane
     * @param red         our alliance is red
     * @param mode        AUTO / forced SCORE / forced PASS
     */
    public Target pick(Translation2d robotCenter, Translation2d launcher, boolean red, Mode mode) {
        return pick(robotCenter, launcher, launcher, red, mode, f.maxPassRange());
    }

    /**
     * @param launcherAhead where the launcher will be when the last ball now in the robot leaves
     *     (a pass lane must be clear from BOTH spots, so it stops BEFORE a hub gets in the way)
     * @param maxRange      longest pass the shooter can make right now (dashboard-tunable)
     */
    public Target pick(Translation2d robotCenter, Translation2d launcher, Translation2d launcherAhead,
                       boolean red, Mode mode, double maxRange) {
        Kind kind;
        if (mode == Mode.SCORE) kind = Kind.SCORE;
        else if (mode == Mode.PASS) kind = Kind.PASS;
        else {
            double d = depth(robotCenter, red);
            // Never AUTO-score past the legal line; come back to SCORE a little inside it.
            if (last == Kind.SCORE) kind = d > f.scoreMaxDepth() ? Kind.PASS : Kind.SCORE;
            else kind = d < f.scoreMaxDepth() - f.hysteresis() ? Kind.SCORE : Kind.PASS;
        }
        last = kind;
        if (kind == Kind.SCORE) return new Target(Kind.SCORE, ourHub(red), "HUB");
        return passTarget(launcher, launcherAhead, red, maxRange);
    }

    /** Corners only, lane checked from one spot, any range (kept for the tests and old callers). */
    public Target passTarget(Translation2d launcher, boolean red) {
        return passTarget(launcher, launcher, red, f.maxPassRange());
    }

    /**
     * Where to pass, best first:
     * <ol>
     *   <li>the corner of our alliance zone on the launcher's side, then its backup, then the far
     *       side's corner and backup (the first that is in range and whose lane is clear);</li>
     *   <li>SHUTTLE: none of those in range (deep in the OPPONENT's alliance zone) = the relay
     *       point in the neutral zone on our half, launcher's side first. A partner (or our own
     *       vacuum mode) passes it home from there. 1678 did the same from the opponent's zone.</li>
     * </ol>
     * A lane counts as clear only if the straight line misses both hubs (plus padding) from the
     * launcher now AND from where it will be when the last ball leaves. All blocked = laneClear
     * false: hold fire (a lob into our hub from outside our zone is a G407 foul, into theirs it
     * scores for them, and either hub eats the ball).
     */
    public Target passTarget(Translation2d launcher, Translation2d ahead, boolean red, double maxRange) {
        boolean lowSide = launcher.getY() < f.width() / 2;
        String[] sideNames = lowSide ? new String[] {"low-y", "high-y"} : new String[] {"high-y", "low-y"};
        boolean[] lows = {lowSide, !lowSide};
        Target first = null;
        boolean anyInRange = false;
        for (int i = 0; i < 2; i++) {
            boolean low = lows[i];
            Translation2d corner = fromOurWall(f.passDepth(), low ? f.passSide() : f.width() - f.passSide(), red);
            Translation2d backup = fromOurWall(f.backupDepth(), low ? f.backupSide() : f.width() - f.backupSide(), red);
            for (int j = 0; j < 2; j++) {
                Translation2d pt = j == 0 ? corner : backup;
                if (!inRange(launcher, ahead, pt, maxRange)) continue;
                anyInRange = true;
                String name = "PASS " + sideNames[i] + (j == 0 ? " corner" : " backup");
                if (first == null) first = new Target(Kind.PASS, pt, name + " (LANE BLOCKED)", false);
                if (laneClear(launcher, ahead, pt, red)) return new Target(Kind.PASS, pt, name);
            }
        }
        if (anyInRange) return first;
        // SHUTTLE: our zone is out of reach, relay into the neutral zone on our half.
        for (int i = 0; i < 2; i++) {
            boolean low = lows[i];
            Translation2d relay = fromOurWall(f.relayDepth(), low ? f.relaySide() : f.width() - f.relaySide(), red);
            String name = "SHUTTLE " + sideNames[i] + " relay (neutral zone)";
            if (first == null) first = new Target(Kind.PASS, relay, name + " (LANE BLOCKED)", false);
            if (laneClear(launcher, ahead, relay, red)) return new Target(Kind.PASS, relay, name);
        }
        return first;
    }

    /** Pass point reachable from the launcher now and where it will be (m). */
    public static boolean inRange(Translation2d launcher, Translation2d ahead, Translation2d pt, double maxRange) {
        return Math.max(launcher.getDistance(pt), ahead.getDistance(pt)) <= maxRange;
    }

    /** Straight line to the point misses both hubs from the launcher now and where it will be. */
    public boolean laneClear(Translation2d launcher, Translation2d ahead, Translation2d pt, boolean red) {
        return !blocked(launcher, pt, red) && !blocked(ahead, pt, red);
    }

    /** True for a SHUTTLE relay target (not one of our alliance zone's corners). */
    public static boolean isRelay(Target t) { return t.name().startsWith("SHUTTLE"); }

    /** Robot (center) is inside the OPPONENT's alliance zone (zoneDepth from their wall). */
    public boolean inTheirZone(Translation2d p, boolean red, double zoneDepth) {
        return depth(p, red) >= f.length() - zoneDepth;
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

    /**
     * TRENCH GUARD: is the hood/Limelight assembly (centered on the turret pivot, reaching
     * `reach` meters around it) inside, or about to be inside, a trench? Each trench is the hub's
     * x band (same depth as the hub, 47 in / 1.194 m) and the opening from the side guardrail
     * (`openingY`, 50.34 in / 1.279 m). Checked at the pivot now and where it will be after
     * `ahead` meters of travel (dx, dy), each grown by `reach`.
     */
    public boolean trenchDanger(Translation2d pivot, double dx, double dy, double reach, double openingY) {
        Translation2d later = pivot.plus(new Translation2d(dx, dy));
        return inTrench(pivot, reach, openingY) || inTrench(later, reach, openingY)
            || inTrench(pivot.interpolate(later, 0.5), reach, openingY);
    }

    private boolean inTrench(Translation2d p, double grow, double openingY) {
        double[] hubs = {f.hubDepth(), f.length() - f.hubDepth()};
        boolean inX = false;
        for (double hx : hubs) inX |= Math.abs(p.getX() - hx) <= f.hubHalfSize() + grow;
        double y = p.getY();
        return inX && (y <= openingY + grow || y >= f.width() - openingY - grow);
    }

    /**
     * Robot (center) is in, or heading into, a trench (under the low bar beside either hub):
     * x within a hub band widened by `approach` meters, y within the trench opening widened by 0.3 m.
     */
    public boolean nearTrench(Translation2d robot, double trenchOpeningY, double approach) {
        double[] hubs = {f.hubDepth(), f.length() - f.hubDepth()};
        boolean inBand = false;
        for (double hx : hubs) inBand |= Math.abs(robot.getX() - hx) <= f.hubHalfSize() + approach;
        double y = robot.getY();
        return inBand && (y <= trenchOpeningY + 0.3 || y >= f.width() - trenchOpeningY - 0.3);
    }
}
