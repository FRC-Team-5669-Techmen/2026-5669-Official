package frc.robot.feeding;

import java.util.ArrayDeque;
import java.util.Locale;

/**
 * Decides WHEN and HOW HARD to agitate the hopper while the robot feeds the shooter. An
 * escalation ladder, cheapest first, reset by every ball that leaves:
 * <ol>
 *   <li>NUDGE (gap of nudgeSeconds): blink the intake roller out/in and shake the spindexer.
 *       Cheap and fast, catches most small bridges before a big gap opens.</li>
 *   <li>PUMP (gap of pumpSeconds): lift the deployed intake part way and drop it, twice, with
 *       the roller blink. If the intake can't pump (stowed, busy, too hot, pump budget used up)
 *       it SHAKEs the spindexer instead.</li>
 *   <li>SHIMMY (still no ball after a pump): shuffle the drivetrain a couple of cm back and forth
 *       for a moment. Only when allowed (dashboard switch, robot standing still, battery OK).</li>
 *   <li>Then PUMP / SHIMMY again until maxTries big agitations in a row bring no ball: the
 *       hopper is probably EMPTY, stop until a ball leaves or feeding restarts.</li>
 * </ol>
 * Repeated spindexer jams (burstCount within burstWindow) also trigger a PUMP.
 * Keeps volley stats (balls, seconds, longest gap, agitations, unjams): the scorecard.
 *
 * <p>Pure logic, no hardware: tested in FeedingTest.
 */
public class AgitationPlanner {
    public enum Action { NONE, NUDGE, PUMP, SHAKE, SHIMMY }

    /** Tunable timings and limits. */
    public static class Params {
        public double nudgeSeconds = 0.35, pumpSeconds = 0.8, cooldown = 0.3, shakeSeconds = 0.15;
        public int maxTries = 3;
        public double burstWindow = 3.0;
        public int burstCount = 3;
    }

    /** What the robot can do right now (filled in by the Agitator every loop). */
    public static class Inputs {
        public boolean enabled, feeding, shooterUp;
        public int shots, jams;
        public boolean pumpPossible;     // intake down, free, cool, pump budget left, battery OK
        public boolean shimmyPossible;   // switch on, driver hands off, battery OK
        public boolean busy;             // a pump or shimmy is still running
    }

    private final Params p;
    private boolean feedingBefore = false, busyBefore = false;
    private double lastProgress = 0, actionEnd = Double.NEGATIVE_INFINITY, lastFeedTime = 0;
    private int level = 0, tries = 0;
    private int lastShots = 0, lastJams = 0;
    private final ArrayDeque<Double> recentJams = new ArrayDeque<>();
    private String status = "idle";

    private boolean volleyOpen = false;
    private double volleyStart = 0, lastBall = 0, longestGap = 0;
    private int volleyBalls = 0, volleyNudges = 0, volleyPumps = 0, volleyShakes = 0, volleyShimmies = 0, volleyJams = 0;
    private String lastVolley = "-";

    public AgitationPlanner(Params p) { this.p = p; }

    public Action update(double now, Inputs in) {
        // ---- bookkeeping ----
        if (in.feeding && !feedingBefore) {
            lastProgress = now; level = 0; tries = 0;
            if (!volleyOpen) {
                volleyOpen = true; volleyStart = now; lastBall = now; longestGap = 0;
                volleyBalls = volleyNudges = volleyPumps = volleyShakes = volleyShimmies = volleyJams = 0;
            }
        }
        if (in.feeding) lastFeedTime = now;
        feedingBefore = in.feeding;
        if (busyBefore && !in.busy) actionEnd = now;
        busyBefore = in.busy;

        int newShots = in.shots - lastShots;
        lastShots = in.shots;
        if (newShots > 0) {
            lastProgress = now; level = 0; tries = 0;
            if (volleyOpen) {
                longestGap = Math.max(longestGap, now - lastBall);
                lastBall = now;
                volleyBalls += newShots;
            }
        }
        int newJams = in.jams - lastJams;
        lastJams = in.jams;
        for (int i = 0; i < newJams; i++) recentJams.addLast(now);
        while (!recentJams.isEmpty() && now - recentJams.peekFirst() > p.burstWindow) recentJams.removeFirst();
        if (volleyOpen) volleyJams += newJams;

        if (volleyOpen && !in.feeding && now - lastFeedTime > 0.5) {
            volleyOpen = false;
            double secs = lastFeedTime - volleyStart;
            lastVolley = String.format(Locale.US,
                "%d balls in %.1f s (%.1f/s), longest gap %.2f s | %d nudges, %d pumps, %d shakes, %d shimmies, %d unjams",
                volleyBalls, secs, secs > 0 ? volleyBalls / secs : 0, longestGap,
                volleyNudges, volleyPumps, volleyShakes, volleyShimmies, volleyJams);
        }

        // ---- decide ----
        if (!in.enabled) { status = "OFF (Auto Agitate switch)"; return Action.NONE; }
        if (!in.feeding) { status = "idle"; return Action.NONE; }
        if (!in.shooterUp) { status = "waiting for shooter"; return Action.NONE; }
        if (in.busy) { status = "agitating"; return Action.NONE; }
        if (now < actionEnd + p.cooldown) { status = "waiting for balls after agitating"; return Action.NONE; }
        if (tries >= p.maxTries) { status = "no balls after " + tries + " tries: hopper EMPTY?"; return Action.NONE; }

        double gap = now - lastProgress;
        boolean jamBurst = recentJams.size() >= p.burstCount;
        if (jamBurst) {
            recentJams.clear();
            return big(now, in, "jams keep coming");
        }
        if (level == 0 && gap > p.nudgeSeconds) {
            level = 1;
            volleyNudges++;
            actionEnd = now + p.shakeSeconds;
            status = "gap: nudge (roller blink + spindexer shake)";
            return Action.NUDGE;
        }
        if (level >= 1 && gap > p.pumpSeconds) {
            return big(now, in, "STARVING");
        }
        status = "feeding OK";
        return Action.NONE;
    }

    /** A big agitation: pump first, shimmy after a pump that didn't work, shake as the fallback. */
    private Action big(double now, Inputs in, String why) {
        tries++;
        lastProgress = now; // judge the gap again from here
        boolean shimmyTurn = level >= 2 && in.shimmyPossible;
        level = Math.max(level, 1) + 1;
        if (shimmyTurn || (!in.pumpPossible && in.shimmyPossible)) {
            volleyShimmies++;
            status = why + ": shimmy the drivetrain";
            return Action.SHIMMY;
        }
        if (in.pumpPossible) {
            volleyPumps++;
            status = why + ": pump intake";
            return Action.PUMP;
        }
        volleyShakes++;
        actionEnd = now + p.shakeSeconds;
        status = why + ": shake spindexer (intake can't pump now)";
        return Action.SHAKE;
    }

    public String status() { return status; }
    /** Gave up: no ball after every try while feeding (hopper probably EMPTY). */
    public boolean probablyEmpty() { return tries >= p.maxTries; }
    public String lastVolley() { return lastVolley; }
}
