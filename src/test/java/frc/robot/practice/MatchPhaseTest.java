package frc.robot.practice;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class MatchPhaseTest {
    @Test
    void phasesFollowTheRebuiltTimeline() {
        assertEquals("AUTO", MatchPhase.phase(true, false, 12).name());
        assertEquals("TRANSITION", MatchPhase.phase(false, true, 135).name());
        assertEquals(5, MatchPhase.phase(false, true, 135).remaining(), 1e-9);
        assertEquals("SHIFT 1", MatchPhase.phase(false, true, 120).name());
        assertEquals("SHIFT 2", MatchPhase.phase(false, true, 90).name());
        assertEquals("SHIFT 3", MatchPhase.phase(false, true, 60).name());
        assertEquals("SHIFT 4", MatchPhase.phase(false, true, 40).name());
        assertEquals("END GAME", MatchPhase.phase(false, true, 10).name());
        assertEquals("DISABLED", MatchPhase.phase(false, false, 10).name());
    }

    @Test
    void autoWinnerIsOffInShiftsOneAndThree() {
        // Red won auto: red off in shift 1 and 3, blue on; flipped in 2 and 4; both on otherwise.
        assertFalse(MatchPhase.hubActive(false, true, 120, true, true));
        assertTrue(MatchPhase.hubActive(false, true, 120, false, true));
        assertTrue(MatchPhase.hubActive(false, true, 90, true, true));
        assertFalse(MatchPhase.hubActive(false, true, 90, false, true));
        assertFalse(MatchPhase.hubActive(false, true, 60, true, true));
        assertTrue(MatchPhase.hubActive(false, true, 40, true, true));
        assertTrue(MatchPhase.hubActive(false, true, 135, true, true));
        assertTrue(MatchPhase.hubActive(false, true, 20, false, true));
        assertTrue(MatchPhase.hubActive(true, false, 10, true, true));
    }

    @Test
    void nextChange() {
        assertEquals(7, MatchPhase.secondsToNextChange(true, 137), 1e-9);
        assertEquals(15, MatchPhase.secondsToNextChange(true, 120), 1e-9);
        assertEquals(-1, MatchPhase.secondsToNextChange(true, 20), 1e-9);
    }
}
