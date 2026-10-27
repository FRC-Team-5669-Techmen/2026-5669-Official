package frc.robot.defense;

/**
 * G418 "There's a 3-count on PINS": a robot may not PIN an opponent for more than 3 s. The robot
 * can't see the other robot, so this counts what it CAN feel: the driver pushing (stick out) while
 * the robot barely moves and the drive motors pull hard. That is either us pinning them or them
 * holding us, and either way the driver should know how long it has lasted.
 *
 * <p>The count keeps running through short breaks (the rule's count only ends after the robots
 * have been apart for more than 3 s), and resets once pushing has stopped for resetSeconds.
 *
 * <p>Pure logic, no hardware: tested in DefenseTest.
 */
public class PinClock {
    private final double stickMin, maxSpeed, minAmps, startSeconds, resetSeconds;
    private double pushStart = Double.NaN;   // when this push count began
    private double pushTime = 0;             // seconds of pushing counted so far
    private double lastPush = Double.NEGATIVE_INFINITY, lastUpdate = Double.NaN;
    private boolean pushing = false;

    /**
     * @param stickMin     driver translate stick (0-1) that counts as "trying to move"
     * @param maxSpeed     robot slower than this (m/s) = not getting anywhere
     * @param minAmps      average drive motor stator current above this = pushing on something
     * @param startSeconds pushing must last this long before it counts (a bump is not a pin)
     * @param resetSeconds not pushing this long = the count starts over
     */
    public PinClock(double stickMin, double maxSpeed, double minAmps, double startSeconds, double resetSeconds) {
        this.stickMin = stickMin;
        this.maxSpeed = maxSpeed;
        this.minAmps = minAmps;
        this.startSeconds = startSeconds;
        this.resetSeconds = resetSeconds;
    }

    /** @return seconds counted so far (0 = not pinning / being pinned) */
    public double update(double now, double stick, double speed, double avgDriveAmps) {
        boolean raw = stick >= stickMin && speed <= maxSpeed && avgDriveAmps >= minAmps;
        double dt = Double.isNaN(lastUpdate) ? 0 : Math.max(0, now - lastUpdate);
        lastUpdate = now;
        if (raw) {
            if (Double.isNaN(pushStart)) pushStart = now;
            lastPush = now;
            if (now - pushStart >= startSeconds) {
                if (!pushing) { pushing = true; pushTime = now - pushStart; }
                else pushTime += dt;
            }
        } else if (now - lastPush > resetSeconds) {
            pushStart = Double.NaN;
            pushing = false;
            pushTime = 0;
        } else if (!pushing) {
            pushStart = Double.NaN; // a short shove that never counted
        }
        return pushing ? pushTime : 0.0;
    }

    /** A pin count is running (pushing now, or only briefly apart since). */
    public boolean pushing() { return pushing; }
}
