package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.junit.jupiter.api.Test;

/**
 * Exercises the pure REBUILT shift-schedule logic across the whole match timeline.
 * Runs on any laptop with ./gradlew test — no robot needed.
 */
class HubShiftTrackerTest {

    /** Blue alliance, blue won AUTO ("B" game data). */
    private boolean blueWonAuto(double matchTime) {
        return HubShiftTracker.computeOurHubOpen(false, true, Alliance.Blue, "B", matchTime);
    }

    /** Blue alliance, red won AUTO ("R" game data). */
    private boolean blueLostAuto(double matchTime) {
        return HubShiftTracker.computeOurHubOpen(false, true, Alliance.Blue, "R", matchTime);
    }

    @Test
    void bothHubsActiveDuringAuto() {
        assertTrue(HubShiftTracker.computeOurHubOpen(true, false, Alliance.Blue, "", 15.0));
        assertTrue(HubShiftTracker.computeOurHubOpen(true, false, null, null, 15.0));
    }

    @Test
    void closedWhenDisabled() {
        assertFalse(HubShiftTracker.computeOurHubOpen(false, false, Alliance.Blue, "B", 100.0));
    }

    @Test
    void closedWithoutFmsData() {
        assertFalse(HubShiftTracker.computeOurHubOpen(false, true, null, "B", 100.0));
        assertFalse(HubShiftTracker.computeOurHubOpen(false, true, Alliance.Blue, "", 100.0));
        assertFalse(HubShiftTracker.computeOurHubOpen(false, true, Alliance.Blue, null, 100.0));
        assertFalse(HubShiftTracker.computeOurHubOpen(false, true, Alliance.Blue, "X", 100.0));
    }

    @Test
    void transitionShiftBothActive() {
        assertTrue(blueWonAuto(135.0));
        assertTrue(blueLostAuto(135.0));
    }

    @Test
    void autoWinnerInactiveInShift1() {
        // SHIFT 1: 2:10-1:45 (130-105 s)
        assertFalse(blueWonAuto(120.0));
        assertTrue(blueLostAuto(120.0));
    }

    @Test
    void shiftsAlternate() {
        // SHIFT 2: 1:45-1:20 (105-80 s)
        assertTrue(blueWonAuto(90.0));
        assertFalse(blueLostAuto(90.0));
        // SHIFT 3: 1:20-0:55 (80-55 s)
        assertFalse(blueWonAuto(70.0));
        assertTrue(blueLostAuto(70.0));
        // SHIFT 4: 0:55-0:30 (55-30 s)
        assertTrue(blueWonAuto(40.0));
        assertFalse(blueLostAuto(40.0));
    }

    @Test
    void endGameBothActive() {
        assertTrue(blueWonAuto(20.0));
        assertTrue(blueLostAuto(20.0));
        assertTrue(blueWonAuto(0.0));
    }

    @Test
    void redAllianceMirrorsBlue() {
        // Red won AUTO -> red hub inactive in SHIFT 1
        assertFalse(HubShiftTracker.computeOurHubOpen(false, true, Alliance.Red, "R", 120.0));
        assertTrue(HubShiftTracker.computeOurHubOpen(false, true, Alliance.Red, "B", 120.0));
    }

    @Test
    void fullGameDataWordsWork() {
        // FMS may send a full word, not just one letter
        assertFalse(HubShiftTracker.computeOurHubOpen(false, true, Alliance.Blue, "Blue", 120.0));
        assertTrue(HubShiftTracker.computeOurHubOpen(false, true, Alliance.Blue, "Red", 120.0));
        assertFalse(HubShiftTracker.computeOurHubOpen(false, true, Alliance.Blue, "blue", 120.0));
    }

    @Test
    void tetheredTeleopWithoutTimerAssumesOpen() {
        // getMatchTime() returns -1 when there is no FMS/practice timer
        assertTrue(blueWonAuto(-1.0));
    }

    @Test
    void shiftBoundariesMatchSchedule() {
        // Boundaries use strict > checks: at exactly 130 s SHIFT 1 has begun,
        // at exactly 30 s END GAME has begun.
        assertFalse(blueWonAuto(130.0)); // SHIFT 1 starts
        assertTrue(blueWonAuto(105.0));  // SHIFT 2 starts
        assertFalse(blueWonAuto(80.0));  // SHIFT 3 starts
        assertTrue(blueWonAuto(55.0));   // SHIFT 4 starts
        assertTrue(blueWonAuto(30.0));   // END GAME starts
    }
}
