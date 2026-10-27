package frc.robot.feeding;

import java.util.ArrayDeque;
import java.util.Locale;

/**
 * Decides WHEN to agitate the hopper while the robot is feeding the shooter.
 *
 * <ul>
 *   <li>STARVING (bridging): feeding with the shooter spun up, but no ball has left for
 *       starveSeconds. Answer: PUMP the intake if it's down and free, else SHAKE the spindexer.</li>
 *   <li>JAM BURST: the spindexer has unjammed burstCount times within burstWindow seconds, so the
 *       pile itself is locked. Answer: PUMP (or SHAKE).</li>
 *   <li>EMPTY: maxPumpsWithoutShot agitations in a row and still no ball: the hopper is probably
 *       empty, so stop agitating until a ball leaves or feeding restarts.</li>
 * </ul>
 * Also keeps volley stats (balls, seconds, longest gap between balls, pumps, unjams): the numbers
 * that show whether agitation works.
 *
 * <p>Pure logic, no hardware: tested in AgitationPlannerTest.
 */
public class AgitationPlanner {
    public enum Action { NONE, PUMP, SHAKE }

    private final double starveSeconds, cooldown, shakeSeconds, burstWindow;
    private final int maxPumpsWithoutShot, burstCount;

    private boolean feedingBefore = false, pumpingBefore = false;
    private double lastProgress = 0, actionEnd = Double.NEGATIVE_INFINITY, lastFeedTime = 0;
    private int pumpsWithoutShot = 0;
    private int lastShots = 0, lastJams = 0;
    private final ArrayDeque<Double> recentJams = new ArrayDeque<>();
    private String status = "idle";

    // volley stats
    private boolean volleyOpen = false;
    private double volleyStart = 0, lastBall = 0, longestGap = 0;
    private int volleyBalls = 0, volleyPumps = 0, volleyShakes = 0, volleyJams = 0;
    private String lastVolley = "-";

    public AgitationPlanner(double starveSeconds, double cooldown, double shakeSeconds,
                            int maxPumpsWithoutShot, double burstWindow, int burstCount) {
        this.starveSeconds = starveSeconds;
        this.cooldown = cooldown;
        this.shakeSeconds = shakeSeconds;
        this.maxPumpsWithoutShot = maxPumpsWithoutShot;
        this.burstWindow = burstWindow;
        this.burstCount = burstCount;
    }

    /**
     * Call every loop.
     *
     * @param now         seconds
     * @param enabled     "Auto Agitate" switch and robot enabled
     * @param feeding     the spindexer is being told to feed forward (firing)
     * @param shooterUp   the flywheel is spinning fast enough to shoot
     * @param shots       ShotDetector total ball count
     * @param jams        JamDetector total jam count
     * @param intakeDown  the ground intake is deployed (a pump is possible)
     * @param pivotFree   nothing else is moving the intake (driver A, jog)
     * @param pumping     a pump this planner asked for is still running
     */
    public Action update(double now, boolean enabled, boolean feeding, boolean shooterUp, int shots, int jams,
                         boolean intakeDown, boolean pivotFree, boolean pumping) {
        // ---- bookkeeping ----
        if (feeding && !feedingBefore) {          // feeding (re)started
            lastProgress = now;
            pumpsWithoutShot = 0;
            if (!volleyOpen) {
                volleyOpen = true;
                volleyStart = now; lastBall = now; longestGap = 0;
                volleyBalls = 0; volleyPumps = 0; volleyShakes = 0; volleyJams = 0;
            }
        }
        if (feeding) lastFeedTime = now;
        feedingBefore = feeding;
        if (pumpingBefore && !pumping) actionEnd = now;  // pump finished: cooldown starts now
        pumpingBefore = pumping;

        int newShots = shots - lastShots;
        lastShots = shots;
        if (newShots > 0) {
            lastProgress = now;
            pumpsWithoutShot = 0;
            if (volleyOpen) {
                longestGap = Math.max(longestGap, now - lastBall);
                lastBall = now;
                volleyBalls += newShots;
            }
        }
        int newJams = jams - lastJams;
        lastJams = jams;
        for (int i = 0; i < newJams; i++) recentJams.addLast(now);
        while (!recentJams.isEmpty() && now - recentJams.peekFirst() > burstWindow) recentJams.removeFirst();
        if (volleyOpen) volleyJams += newJams;

        // volley ends half a second after feeding stops
        if (volleyOpen && !feeding && now - lastFeedTime > 0.5) {
            volleyOpen = false;
            double secs = lastFeedTime - volleyStart;
            lastVolley = String.format(Locale.US,
                "%d balls in %.1f s (%.1f/s), longest gap %.2f s, %d pumps, %d shakes, %d unjams",
                volleyBalls, secs, secs > 0 ? volleyBalls / secs : 0, longestGap, volleyPumps, volleyShakes, volleyJams);
        }

        // ---- decide ----
        if (!enabled) { status = "OFF (Auto Agitate switch)"; return Action.NONE; }
        if (!feeding) { status = "idle"; return Action.NONE; }
        if (!shooterUp) { status = "waiting for shooter"; return Action.NONE; }
        if (pumping) { status = "PUMPING intake"; return Action.NONE; }
        if (now < actionEnd + cooldown) { status = "waiting for balls after agitating"; return Action.NONE; }
        if (pumpsWithoutShot >= maxPumpsWithoutShot) { status = "no balls after " + pumpsWithoutShot + " tries: hopper EMPTY?"; return Action.NONE; }

        boolean starving = now - lastProgress > starveSeconds;
        boolean jamBurst = recentJams.size() >= burstCount;
        if (!starving && !jamBurst) { status = "feeding OK"; return Action.NONE; }

        if (jamBurst) recentJams.clear();
        pumpsWithoutShot++;
        lastProgress = now; // judge starving again from the end of this agitation
        if (intakeDown && pivotFree) {
            volleyPumps++;
            status = (jamBurst ? "jams keep coming" : "STARVING") + ": pump intake";
            return Action.PUMP;
        }
        volleyShakes++;
        actionEnd = now + shakeSeconds;
        status = (jamBurst ? "jams keep coming" : "STARVING") + ": shake spindexer (intake not down/free)";
        return Action.SHAKE;
    }

    public String status() { return status; }
    public String lastVolley() { return lastVolley; }
}
