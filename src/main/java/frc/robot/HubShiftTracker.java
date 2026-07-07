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
 */
public final class HubShiftTracker {

    private HubShiftTracker() {}

    /**
     * @return true when scoring FUEL into our HUB earns points right now.
     *         Returns false when the alliance or game data is unknown (e.g. practice
     *         without FMS), so the rumble/dashboard cues stay quiet instead of lying.
     */
    public static boolean isOurHubOpen() {
        if (DriverStation.isAutonomousEnabled()) {
            return true; // both HUBs are active for all of AUTO
        }
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        var alliance = DriverStation.getAlliance();
        String gameData = DriverStation.getGameSpecificMessage();
        if (alliance.isEmpty() || gameData == null || gameData.isEmpty()) {
            return false; // no FMS data — unknown, treat as closed
        }

        boolean weWonAuto;
        switch (Character.toUpperCase(gameData.charAt(0))) {
            case 'R' -> weWonAuto = alliance.get() == Alliance.Red;
            case 'B' -> weWonAuto = alliance.get() == Alliance.Blue;
            default -> {
                return false; // unrecognized game data
            }
        }

        double matchTime = DriverStation.getMatchTime();
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
}
