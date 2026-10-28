package frc.robot.power;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * BATTERY ESTIMATOR (Oct 28): finds the resistance of the battery plus the robot's main wiring
 * from how far the voltage drops when the current rises (V = Voc - I x R), the way a battery
 * tester does it, but using the robot's own load. Pure math (no WPILib), tested in
 * BatteryEstimatorTest.
 *
 * <p>Why it tells battery from robot: R is the battery's internal resistance PLUS the main
 * leads, Anderson connector, 120 A breaker and PDH lugs. Every battery on the same robot shares
 * the wiring part, so: one battery reads high and the others normal = that battery; every
 * battery reads high = the robot's wiring. A normal R with a deep sag = the robot simply pulled
 * a lot of current (drive pushing, shooter spin-up, stalls).
 *
 * <p>Method: a sliding 2 s window of (current, voltage) samples. Every 0.5 s, if the current
 * moved over at least {@link #kMinAmpSpread} A inside the window, a least-squares line through
 * the points gives R (slope) and Voc (intercept). Fits that are noisy (r^2 under 0.6) or
 * impossible (R outside 3 to 200 mOhm) are thrown away. The answer is the median of the good
 * fits, so one odd window can't move it.
 */
public final class BatteryEstimator {
    public static final double kWindowSeconds = 2.0;
    public static final double kMinAmpSpread = 25.0;
    public static final double kFitEverySeconds = 0.5;
    public static final double kMinR = 0.003, kMaxR = 0.200;
    public static final double kMinR2 = 0.6;

    private record S(double t, double v, double i) {}

    private final ArrayDeque<S> window = new ArrayDeque<>();
    private final List<Double> rFits = new ArrayList<>();
    private final List<Double> vocFits = new ArrayList<>();
    private double lastFit = Double.NEGATIVE_INFINITY;
    private double lastR = Double.NaN;

    /** One loop's battery voltage (V) and total robot current (A) at time t (s). */
    public void add(double t, double volts, double amps) {
        if (!(volts > 1) || !(amps >= 0)) return;
        window.addLast(new S(t, volts, amps));
        while (!window.isEmpty() && t - window.peekFirst().t() > kWindowSeconds) window.removeFirst();
        if (t - lastFit < kFitEverySeconds) return;
        lastFit = t;
        fit();
    }

    private void fit() {
        int n = window.size();
        if (n < 15) return;
        double minI = Double.MAX_VALUE, maxI = -1, sx = 0, sy = 0;
        for (S s : window) { minI = Math.min(minI, s.i()); maxI = Math.max(maxI, s.i()); sx += s.i(); sy += s.v(); }
        if (maxI - minI < kMinAmpSpread) return;
        double mx = sx / n, my = sy / n, sxx = 0, sxy = 0, syy = 0;
        for (S s : window) {
            double dx = s.i() - mx, dy = s.v() - my;
            sxx += dx * dx; sxy += dx * dy; syy += dy * dy;
        }
        if (sxx <= 0 || syy <= 0) return;
        double slope = sxy / sxx;          // volts per amp, negative when the battery sags
        double r2 = (sxy * sxy) / (sxx * syy);
        double r = -slope;
        if (r < kMinR || r > kMaxR || r2 < kMinR2) return;
        lastR = r;
        rFits.add(r);
        vocFits.add(my - slope * mx);
    }

    /** Median R (ohms) over every good fit so far, NaN until there is one. */
    public double resistance() { return median(rFits); }

    /** Median open-circuit voltage (V) the fits point to, NaN until there is one. */
    public double openCircuitVolts() { return median(vocFits); }

    /** The latest good fit (ohms), NaN until there is one. */
    public double latest() { return lastR; }

    public int fits() { return rFits.size(); }

    public void reset() { window.clear(); rFits.clear(); vocFits.clear(); lastFit = Double.NEGATIVE_INFINITY; lastR = Double.NaN; }

    static double median(List<Double> xs) {
        if (xs.isEmpty()) return Double.NaN;
        List<Double> c = new ArrayList<>(xs);
        Collections.sort(c);
        int m = c.size() / 2;
        return c.size() % 2 == 1 ? c.get(m) : (c.get(m - 1) + c.get(m)) / 2.0;
    }

    /**
     * Plain-English verdict for one match or test. restVolts = before the match, disabled
     * (NaN if unknown); r in ohms (NaN if no fit); minVolts and ampsAtMin at the lowest point.
     */
    public static String verdict(double restVolts, double r, double minVolts, double ampsAtMin, int brownouts,
                                 double rHigh, double restLow) {
        List<String> out = new ArrayList<>();
        if (!Double.isNaN(restVolts) && restVolts < restLow)
            out.add(String.format("CHARGE: started at %.2f V (fresh is 12.7+, %.1f minimum)", restVolts, restLow));
        if (!Double.isNaN(r) && r > rHigh)
            out.add(String.format("HIGH RESISTANCE %.0f mOhm (normal under %.0f): worn battery OR a loose main connection. "
                + "Only this battery reads high = the battery. Every battery high = robot wiring (Anderson, breaker, PDH lugs).",
                r * 1000, rHigh * 1000));
        if (brownouts > 0 && (Double.isNaN(r) || r <= rHigh) && ampsAtMin > 200)
            out.add(String.format("ROBOT DRAW: %.0f A at the dip with normal resistance: too much current at once "
                + "(drive pushing, shooter spin-up, a stall)", ampsAtMin));
        if (brownouts > 0 && out.isEmpty())
            out.add(String.format("BROWNOUT at %.1f V, %.0f A: cause unclear, check the log", minVolts, ampsAtMin));
        return out.isEmpty() ? "OK" : String.join(" | ", out);
    }
}
