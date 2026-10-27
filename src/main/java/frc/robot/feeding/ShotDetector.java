package frc.robot.feeding;

/**
 * Counts balls leaving the shooter from the flywheel speed alone. Each ball pulls the wheel down
 * for a moment; a ball = the speed falls kDrop below its recent peak. Works at any target, even
 * one the wheel can't reach, because it compares the wheel against itself (ShooterSubsystem's
 * meter compares against the target and is for "launched at speed?", not for counting).
 *
 * <p>Pure math, no hardware: tested in ShotDetectorTest.
 */
public class ShotDetector {
    private final double drop, rearm, maxDipSeconds, minRpm;
    private double peak = 0, launch = 0, dipStart = 0;
    private boolean inDip = false;
    private int shots = 0;
    private double lastShotTime = Double.NEGATIVE_INFINITY;

    public ShotDetector(double dropRpm, double rearmRpm, double maxDipSeconds, double minRpm) {
        this.drop = dropRpm;
        this.rearm = rearmRpm;
        this.maxDipSeconds = maxDipSeconds;
        this.minRpm = minRpm;
    }

    /** Call every loop. Returns true on the loop a new ball is detected. */
    public boolean update(double now, double rpm) {
        if (rpm < minRpm) {          // shooter not spinning: nothing to count
            inDip = false;
            peak = rpm;
            return false;
        }
        if (inDip) {
            if (rpm >= launch - rearm || now - dipStart > maxDipSeconds) {
                inDip = false;
                peak = rpm;          // start the next peak from here
            }
            return false;
        }
        if (rpm > peak) peak = rpm;  // spinning up or holding: follow it
        if (rpm < peak - drop) {
            inDip = true;
            launch = peak;
            dipStart = now;
            shots++;
            lastShotTime = now;
            return true;
        }
        return false;
    }

    public int shots() { return shots; }
    public double lastShotTime() { return lastShotTime; }
}
