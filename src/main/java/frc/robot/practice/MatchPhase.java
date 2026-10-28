package frc.robot.practice;

/**
 * REBUILT match timeline (game manual TU22), pure logic so it can be unit tested and shared by the
 * robot's practice-match display. Times are the Driver Station's match time = seconds LEFT in the
 * current period (auto 20 s, teleop 140 s).
 *
 * <pre>
 * AUTO          20 s  both hubs active
 * TRANSITION    10 s  (teleop 140-130) both active
 * SHIFT 1-4  4x25 s  (130-30) alternate: the auto WINNER's hub is inactive in shifts 1 and 3
 * END GAME      30 s  (30-0) both active
 * </pre>
 */
public final class MatchPhase {
    private MatchPhase() {}

    public record Phase(String name, double remaining) {}

    /** Teleop boundaries (match time left) where the phase changes. */
    private static final double[] kTeleopBounds = {130, 105, 80, 55, 30, 0};
    private static final String[] kTeleopNames = {"TRANSITION", "SHIFT 1", "SHIFT 2", "SHIFT 3", "SHIFT 4", "END GAME"};

    /** Current phase and seconds left in it (remaining -1 = unknown). */
    public static Phase phase(boolean auto, boolean teleop, double matchTime) {
        if (auto) return new Phase("AUTO", matchTime);
        if (!teleop) return new Phase("DISABLED", -1);
        if (matchTime < 0) return new Phase("TELEOP (no match clock)", -1);
        for (int i = 0; i < kTeleopBounds.length; i++) {
            if (matchTime > kTeleopBounds[i]) return new Phase(kTeleopNames[i], matchTime - kTeleopBounds[i]);
        }
        return new Phase("END GAME", 0);
    }

    /**
     * Is this alliance's hub active? `redWonAuto` null = auto winner unknown (both treated active
     * in shifts, as the robot code does without game data).
     */
    public static boolean hubActive(boolean auto, boolean teleop, double matchTime, boolean red, Boolean redWonAuto) {
        if (auto) return true;
        if (!teleop) return false;
        if (matchTime < 0 || redWonAuto == null) return true;
        boolean weWon = red == redWonAuto;
        if (matchTime > 130) return true;
        if (matchTime > 105) return !weWon; // shift 1: the auto winner's hub is OFF
        if (matchTime > 80) return weWon;
        if (matchTime > 55) return !weWon;
        if (matchTime > 30) return weWon;
        return true;
    }

    /** Seconds until any hub changes state in teleop, or -1 (auto, disabled, end game, unknown). */
    public static double secondsToNextChange(boolean teleop, double matchTime) {
        if (!teleop || matchTime < 0) return -1;
        for (double b : new double[] {130, 105, 80, 55, 30}) if (matchTime > b) return matchTime - b;
        return -1;
    }
}
