package frc.robot.characterization;

/**
 * DRIVE FEEDFORWARD from a slow voltage ramp (6328's "feedforward characterization"): the robot
 * drives straight while the voltage climbs slowly, and a straight line through (speed, voltage)
 * gives kS (volts to get moving) and kV (volts per m/s). Least squares over every sample where
 * the robot is actually moving.
 *
 * <p>Pure math, no hardware: tested in CharacterizationTest.
 */
public class FeedforwardFit {
    private double n, sx, sy, sxx, sxy;
    private final double minSpeed;

    /** @param minSpeed samples slower than this (m/s) are ignored (static friction region) */
    public FeedforwardFit(double minSpeed) { this.minSpeed = minSpeed; }

    public void add(double volts, double speed) {
        if (Math.abs(speed) < minSpeed) return;
        double x = Math.abs(speed), y = Math.abs(volts);
        n++; sx += x; sy += y; sxx += x * x; sxy += x * y;
    }

    public int samples() { return (int) n; }

    /** Volts per m/s (NaN until there are enough moving samples). */
    public double kV() {
        double d = n * sxx - sx * sx;
        return n < 10 || Math.abs(d) < 1e-9 ? Double.NaN : (n * sxy - sx * sy) / d;
    }

    /** Volts to overcome friction. */
    public double kS() {
        double kv = kV();
        return Double.isNaN(kv) ? Double.NaN : (sy - kv * sx) / n;
    }

    /** kV in the Talon's units: volts per MOTOR rotation per second. */
    public static double perMotorRps(double voltsPerMps, double wheelRadius, double gearRatio) {
        return voltsPerMps * 2 * Math.PI * wheelRadius / gearRatio;
    }
}
