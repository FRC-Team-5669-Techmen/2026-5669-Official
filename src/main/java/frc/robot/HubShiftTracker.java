package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Computes whether our HUB is currently active under the REBUILT 2026 shift schedule.
 *
 * <p>FMS broadcasts the AUTO winner in the game specific message at TELEOP start
 * ("R..." = red won AUTO, "B..." = blue won AUTO). The AUTO winner's HUB goes
 * INACTIVE in SHIFT 1 and the two HUBs alternate every 25-second shift. Both HUBs
 * are active during AUTO, the TRANSITION SHIFT, and END GAME.
 *
 * <p>Shift schedule (teleop timer counts down from 2:20):
 * <pre>
 * TRANSITION  2:20-2:10  both HUBs active
 * SHIFT 1     2:10-1:45  AUTO winner inactive
 * SHIFT 2     1:45-1:20  flipped
 * SHIFT 3     1:20-0:55  same as SHIFT 1
 * SHIFT 4     0:55-0:30  same as SHIFT 2
 * END GAME    0:30-0:00  both HUBs active
 * </pre>
 *
 * <p>The decision logic lives in {@link #computeOurHubOpen} as a pure function so
 * it can be unit tested without a robot (see HubShiftTrackerTest).
 */
public final class HubShiftTracker {

    private HubShiftTracker() {}

    // ---------------------------------------------------------------- AUTO WINNER OVERRIDE
    // FMS game data ('R'/'B' = the alliance that won AUTO) drives the shift schedule. If it never
    // arrives (a missed message is not an arena fault), the drive team picks it on the dashboard
    // chooser "Auto Winner" (6328, 1678 and 5000 all had this override in 2026).
    public enum AutoWinner { FMS, WE_WON, THEY_WON }
    private static volatile AutoWinner override = AutoWinner.FMS;

    public static void setAutoWinnerOverride(AutoWinner w) { override = w == null ? AutoWinner.FMS : w; }

    /** True when the FMS sent no game data (and no override is set): rumble both controllers. */
    public static boolean gameDataMissing() {
        String g = DriverStation.getGameSpecificMessage();
        return (g == null || g.isEmpty()) && override == AutoWinner.FMS;
    }

    /**
     * FMS game data; the override is used ONLY while the FMS has sent nothing (audit Oct 27: a
     * dashboard re-sends its last chooser pick after a reboot, so a pick left over from an earlier
     * match would have overridden real FMS data and held fire in an open hub for half of teleop).
     */
    public static String gameData() {
        String g = DriverStation.getGameSpecificMessage();
        if (override == AutoWinner.FMS || (g != null && !g.isEmpty())) return g;
        var a = DriverStation.getAlliance();
        if (a.isEmpty()) return g;
        boolean weAreRed = a.get() == Alliance.Red;
        boolean redWon = override == AutoWinner.WE_WON ? weAreRed : !weAreRed;
        return redWon ? "R" : "B";
    }

    /**
     * @return true when scoring FUEL into our HUB earns points right now.
     *         Returns false when the alliance or game data is unknown (e.g. practice
     *         without FMS), so the rumble/dashboard cues stay quiet instead of lying.
     */
    public static boolean isOurHubOpen() {
        return computeOurHubOpen(
            DriverStation.isAutonomousEnabled(),
            DriverStation.isTeleopEnabled(),
            DriverStation.getAlliance().orElse(null),
            gameData(),
            DriverStation.getMatchTime());
    }

    /**
     * Pure shift-schedule logic — no DriverStation reads, so it is unit testable.
     *
     * @param autonomousEnabled true while the robot is enabled in autonomous
     * @param teleopEnabled     true while the robot is enabled in teleop
     * @param alliance          our alliance color, or null if unknown
     * @param gameData          FMS game specific message ("R..." / "B..." = AUTO winner)
     * @param matchTime         seconds remaining in the current period (negative if unknown)
     */
    public static boolean computeOurHubOpen(
            boolean autonomousEnabled,
            boolean teleopEnabled,
            Alliance alliance,
            String gameData,
            double matchTime) {
        if (autonomousEnabled) {
            return true; // both HUBs are active for all of AUTO
        }
        if (!teleopEnabled) {
            return false;
        }
        if (alliance == null || gameData == null || gameData.isEmpty()) {
            return false; // no FMS data — unknown, treat as closed
        }

        boolean weWonAuto;
        switch (Character.toUpperCase(gameData.charAt(0))) {
            case 'R' -> weWonAuto = alliance == Alliance.Red;
            case 'B' -> weWonAuto = alliance == Alliance.Blue;
            default -> {
                return false; // unrecognized game data
            }
        }

        if (matchTime < 0) {
            return true; // no usable timer (tethered teleop) — assume open
        }

        if (matchTime > 130) return true;       // TRANSITION SHIFT: both active
        if (matchTime > 105) return !weWonAuto; // SHIFT 1
        if (matchTime > 80)  return weWonAuto;  // SHIFT 2
        if (matchTime > 55)  return !weWonAuto; // SHIFT 3
        if (matchTime > 30)  return weWonAuto;  // SHIFT 4
        return true;                            // END GAME: both active
    }

    /** OPEN / CLOSED, or UNKNOWN without FMS game data (practice): never block shots on UNKNOWN. */
    public enum HubState { OPEN, CLOSED, UNKNOWN }

    /**
     * Our hub's state when a ball fired NOW will arrive (secondsAhead = flight time + a margin),
     * so shooting can start just before the hub opens and stop just before it closes
     * (6328 and 581 did this in 2026).
     */
    public static HubState ourHubStateAt(double secondsAhead) {
        return computeHubStateAt(
            DriverStation.isAutonomousEnabled(),
            DriverStation.isTeleopEnabled(),
            DriverStation.getAlliance().orElse(null),
            gameData(),
            DriverStation.getMatchTime(),
            secondsAhead);
    }

    /** Pure version of {@link #ourHubStateAt}, unit tested. */
    public static HubState computeHubStateAt(boolean autonomousEnabled, boolean teleopEnabled, Alliance alliance,
                                             String gameData, double matchTime, double secondsAhead) {
        if (autonomousEnabled) return HubState.OPEN;
        if (!teleopEnabled || alliance == null || gameData == null || gameData.isEmpty() || matchTime < 0) {
            return HubState.UNKNOWN;
        }
        char c = Character.toUpperCase(gameData.charAt(0));
        if (c != 'R' && c != 'B') return HubState.UNKNOWN;
        double t = matchTime - Math.max(0.0, secondsAhead);
        if (t < 0) t = 0;
        // The hub keeps counting FUEL for 3 s after it deactivates (game manual): a ball that
        // arrives up to kCloseGrace after closing still scores.
        boolean open = computeOurHubOpen(false, true, alliance, gameData, t)
            || computeOurHubOpen(false, true, alliance, gameData, t + kCloseGrace);
        return open ? HubState.OPEN : HubState.CLOSED;
    }

    /** Seconds after deactivating that the hub still counts FUEL: 3 s by rule, 2.5 s to be safe. */
    public static final double kCloseGrace = 2.5;

    /** Seconds until our hub next changes state (teleop, FMS data known), or -1 if unknown. */
    public static double secondsUntilChange(double matchTime) {
        if (matchTime < 0) return -1;
        double[] bounds = {130, 105, 80, 55, 30};
        for (double b : bounds) if (matchTime > b) return matchTime - b;
        return -1;
    }
}
