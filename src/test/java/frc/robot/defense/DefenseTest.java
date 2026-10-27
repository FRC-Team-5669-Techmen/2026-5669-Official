package frc.robot.defense;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class DefenseTest {
    static PinClock clock() { return new PinClock(0.5, 0.25, 40, 0.5, 3.0); }

    @Test
    void pushingIntoSomethingCountsAfterAMoment() {
        PinClock c = clock();
        double t = 0;
        for (; t < 0.45; t += 0.02) assertEquals(0.0, c.update(t, 1.0, 0.05, 60), 1e-9);
        for (; t < 2.0; t += 0.02) c.update(t, 1.0, 0.05, 60);
        assertEquals(2.0, c.update(2.0, 1.0, 0.05, 60), 0.05);
        assertTrue(c.pushing());
    }

    @Test
    void drivingFreelyOrLightContactNeverCounts() {
        PinClock c = clock();
        for (double t = 0; t < 3; t += 0.02) {
            assertEquals(0.0, c.update(t, 1.0, 3.0, 80), 1e-9);  // moving fast
        }
        for (double t = 3; t < 6; t += 0.02) {
            assertEquals(0.0, c.update(t, 1.0, 0.0, 10), 1e-9);  // low current (driver against nothing)
        }
        for (double t = 6; t < 9; t += 0.02) {
            assertEquals(0.0, c.update(t, 0.1, 0.0, 80), 1e-9);  // stick released
        }
    }

    @Test
    void shortBreaksDoNotResetTheCountButALongOneDoes() {
        PinClock c = clock();
        double t = 0;
        for (; t < 2.0; t += 0.02) c.update(t, 1.0, 0.0, 60);
        for (; t < 3.0; t += 0.02) c.update(t, 0.0, 0.0, 0);          // backed off 1 s
        for (; t < 3.5; t += 0.02) c.update(t, 1.0, 0.0, 60);         // back in
        assertTrue(c.update(3.5, 1.0, 0.0, 60) > 2.0);
        for (; t < 7.0; t += 0.02) c.update(t, 0.0, 0.0, 0);          // apart 3.5 s
        assertEquals(0.0, c.update(7.0, 0.0, 0.0, 0), 1e-9);
    }

    @Test
    void towerGuardWatchesTheOpponentsTower() {
        // Blue robot near the red tower (red wall, x near 16.5, y 4.32).
        assertTrue(TowerGuard.nearTheirTower(15.8, 4.3, false, 0.9));
        assertFalse(TowerGuard.nearTheirTower(0.8, 3.7, false, 0.9));   // our own tower: no rule
        assertFalse(TowerGuard.nearTheirTower(15.8, 1.0, false, 0.9));  // far side of their wall
        // Red robot near the blue tower.
        assertTrue(TowerGuard.nearTheirTower(0.9, 3.7, true, 0.9));
        assertTrue(TowerGuard.endgame(29.0));
        assertFalse(TowerGuard.endgame(45.0));
        assertFalse(TowerGuard.endgame(0.0));
    }
}
