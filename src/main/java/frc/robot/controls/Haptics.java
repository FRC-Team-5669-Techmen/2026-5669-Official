package frc.robot.controls;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * HAPTICS (Oct 28): every mode change is FELT, so a drive team that can't look at a screen always
 * knows what the robot is doing (hidden mode state is the classic cause of "mode errors").
 *
 * <p>One vocabulary for both controllers, kept small on purpose:
 * <ul>
 *   <li>ONE short buzz = something turned ON (auto-aim on, gyro reset done, backup spot set).</li>
 *   <li>TWO short buzzes = something turned OFF.</li>
 *   <li>ONE long buzz = a hands-free mode started (vacuum or dump: the robot will move things).</li>
 *   <li>1 / 2 / 3 short = shot target mode AUTO / SCORE only / PASS only (Operator Start).</li>
 *   <li>THREE long = hopper looks EMPTY (go get balls).</li>
 * </ul>
 * A pattern plays for under 1.5 s and wins over the steady "ready to fire" buzz while it plays.
 * Teleop only (in auto and disabled nobody is holding the controllers).
 */
public final class Haptics {
    public enum Pad { DRIVER, OPERATOR }

    private static final double[] start = {Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY};
    private static final double[][] pattern = {new double[0], new double[0]}; // on/off durations

    private Haptics() {}

    private static void play(Pad pad, double... onOff) {
        if (!DriverStation.isTeleopEnabled()) return;
        start[pad.ordinal()] = Timer.getFPGATimestamp();
        pattern[pad.ordinal()] = onOff;
    }

    /** n short buzzes (0.12 s on, 0.12 s off). */
    public static void shortBuzzes(Pad pad, int n) {
        double[] p = new double[Math.max(1, n) * 2];
        for (int i = 0; i < p.length; i++) p[i] = 0.12;
        play(pad, p);
    }

    public static void on(Pad pad) { shortBuzzes(pad, 1); }
    public static void off(Pad pad) { shortBuzzes(pad, 2); }
    public static void longBuzz(Pad pad) { play(pad, 0.6, 0.1); }
    public static void empty(Pad pad) { play(pad, 0.35, 0.15, 0.35, 0.15, 0.35, 0.1); }

    /** Rumble strength (0-1) the pattern wants now, or -1 when no pattern is playing. */
    public static double level(Pad pad) {
        double t = Timer.getFPGATimestamp() - start[pad.ordinal()];
        double[] p = pattern[pad.ordinal()];
        if (t < 0) return -1;
        for (int i = 0; i < p.length; i++) {
            if (t < p[i]) return i % 2 == 0 ? 1.0 : 0.0;
            t -= p[i];
        }
        return -1;
    }
}
