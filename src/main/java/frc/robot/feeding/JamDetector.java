package frc.robot.feeding;

/**
 * Spindexer INLET JAM: two balls fight for the uptake and the spindexer stalls. Seen as: told to
 * feed forward, pushing hard (stator amps) but barely turning, for a moment. Answer: reverse for
 * a moment (unjam), then feed again. Also asks for a shake on demand (Agitator, intake stowed).
 *
 * <p>Pure math, no hardware: tested in FeedingTest.
 */
public class JamDetector {
    private final double jamAmps, jamSpeed, jamSeconds, ignoreAfterStart, unjamSeconds;
    private final double burstWindow, lockoutSeconds;
    private final int burstCount;
    private final java.util.ArrayDeque<Double> recent = new java.util.ArrayDeque<>();
    private double lockedUntil = Double.NEGATIVE_INFINITY;
    private boolean feeding = false;
    private double feedStart = 0, stallSince = Double.NaN, reverseUntil = Double.NEGATIVE_INFINITY;
    private int jams = 0;

    public JamDetector(double jamAmps, double jamSpeed, double jamSeconds, double ignoreAfterStart, double unjamSeconds) {
        this(jamAmps, jamSpeed, jamSeconds, ignoreAfterStart, unjamSeconds, 3.0, 3, 3.0);
    }

    /**
     * @param burstWindow/burstCount/lockoutSeconds burstCount jams within burstWindow = stop
     *     auto-reversing for lockoutSeconds (spares the gearbox from hammering back and forth)
     */
    public JamDetector(double jamAmps, double jamSpeed, double jamSeconds, double ignoreAfterStart, double unjamSeconds,
                       double burstWindow, int burstCount, double lockoutSeconds) {
        this.burstWindow = burstWindow;
        this.burstCount = burstCount;
        this.lockoutSeconds = lockoutSeconds;
        this.jamAmps = jamAmps;
        this.jamSpeed = jamSpeed;
        this.jamSeconds = jamSeconds;
        this.ignoreAfterStart = ignoreAfterStart;
        this.unjamSeconds = unjamSeconds;
    }

    /**
     * Call every loop while the spindexer is told to feed forward.
     *
     * @return true = drive the spindexer BACKWARDS this loop (unjamming or shaking)
     */
    public boolean update(double now, double amps, double speed) {
        if (!feeding) {
            feeding = true;
            feedStart = now;
            stallSince = Double.NaN;
        }
        if (now < reverseUntil) return true;
        boolean stalled = Math.abs(amps) > jamAmps && Math.abs(speed) < jamSpeed
            && now - feedStart > ignoreAfterStart;
        if (!stalled) {
            stallSince = Double.NaN;
            return false;
        }
        if (Double.isNaN(stallSince)) stallSince = now;
        if (now - stallSince >= jamSeconds) {
            if (now < lockedUntil) return false; // too many in a row: let the stator limit hold it
            jams++;
            recent.addLast(now);
            while (!recent.isEmpty() && now - recent.peekFirst() > burstWindow) recent.removeFirst();
            if (recent.size() >= burstCount) { lockedUntil = now + unjamSeconds + lockoutSeconds; recent.clear(); }
            reverseUntil = now + unjamSeconds;
            stallSince = Double.NaN;
            // after reversing, give it a fresh spin-up before judging again
            feedStart = reverseUntil;
            return true;
        }
        return false;
    }

    /** Feeding stopped (or reversed by hand): forget the stall timers. */
    public void stopped() {
        feeding = false;
        stallSince = Double.NaN;
        reverseUntil = Double.NEGATIVE_INFINITY;
    }

    /** Reverse for this long starting now (a shake), then feed again. */
    public void shake(double now, double seconds) {
        reverseUntil = Math.max(reverseUntil, now + seconds);
        feedStart = reverseUntil;
    }

    public boolean reversing(double now) { return now < reverseUntil; }
    public boolean lockedOut(double now) { return now < lockedUntil; }
    public int jams() { return jams; }
}
