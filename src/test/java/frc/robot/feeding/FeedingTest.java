package frc.robot.feeding;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

/** Scenarios for automatic agitation, run at the robot's 50 Hz loop. */
class FeedingTest {
    static final double DT = 0.02;

    // ---------------- ShotDetector ----------------

    @Test
    void countsEachBallDipEvenBelowTarget() {
        ShotDetector d = new ShotDetector(120, 60, 0.6, 1000);
        double t = 0;
        // spin up to 4400 (a wheel that can't reach a 5400 target): no false shots
        for (double rpm = 0; rpm <= 4400; rpm += 200, t += DT) assertFalse(d.update(t, rpm));
        for (int i = 0; i < 20; i++, t += DT) d.update(t, 4400);
        assertEquals(0, d.shots());
        // 3 balls: dip 300, recover over 0.2 s
        for (int ball = 0; ball < 3; ball++) {
            d.update(t, 4100); t += DT;
            for (int i = 0; i < 10; i++, t += DT) d.update(t, 4100 + 30 * i);
            for (int i = 0; i < 10; i++, t += DT) d.update(t, 4400);
        }
        assertEquals(3, d.shots());
    }

    @Test
    void ignoresSmallNoise() {
        ShotDetector d = new ShotDetector(120, 60, 0.6, 1000);
        double t = 0;
        for (int i = 0; i < 200; i++, t += DT) d.update(t, 4000 + ((i % 2 == 0) ? 40 : -40));
        assertEquals(0, d.shots());
    }

    // ---------------- JamDetector ----------------

    @Test
    void stallReversesThenFeedsAgain() {
        JamDetector j = new JamDetector(45, 8, 0.15, 0.3, 0.25);
        double t = 0;
        for (int i = 0; i < 25; i++, t += DT) assertFalse(j.update(t, 20, 40)); // feeding fine
        // stalled until it reverses (about 0.15 s), then the jam is free
        int reverseLoops = 0;
        while (!j.update(t, 70, 1)) { t += DT; assertTrue(t < 1.5, "a stall must trigger a reverse"); }
        reverseLoops++; t += DT;
        assertEquals(1, j.jams());
        // stays reversed ~0.25 s in total, then feeds forward again
        for (int i = 0; i < 30; i++, t += DT) if (j.update(t, 20, 40)) reverseLoops++;
        assertTrue(reverseLoops >= 10 && reverseLoops <= 14, "reverse lasted " + reverseLoops + " loops");
    }

    @Test
    void spinUpSurgeIsNotAJam() {
        JamDetector j = new JamDetector(45, 8, 0.15, 0.3, 0.25);
        double t = 0;
        for (int i = 0; i < 12; i++, t += DT) assertFalse(j.update(t, 75, 2)); // first 0.24 s
        assertEquals(0, j.jams());
    }

    // ---------------- AgitationPlanner ----------------

    static AgitationPlanner planner() { return new AgitationPlanner(0.8, 0.6, 0.15, 3, 3.0, 3); }

    @Test
    void steadyFlowNeverAgitates() {
        AgitationPlanner p = planner();
        double t = 0; int shots = 0;
        for (int i = 0; i < 250; i++, t += DT) {
            if (i % 12 == 0) shots++; // a ball every 0.24 s
            assertEquals(AgitationPlanner.Action.NONE, p.update(t, true, true, true, shots, 0, true, true, false));
        }
    }

    @Test
    void bridgedPilePumpsTheIntake() {
        AgitationPlanner p = planner();
        double t = 0; int shots = 0; int pumpsAt = -1;
        for (int i = 0; i < 100; i++, t += DT) {
            if (i < 20 && i % 10 == 0) shots++;         // two balls, then nothing (bridged)
            AgitationPlanner.Action a = p.update(t, true, true, true, shots, 0, true, true, false);
            if (a == AgitationPlanner.Action.PUMP) { pumpsAt = i; break; }
        }
        assertTrue(pumpsAt > 0, "must pump when starving");
        // last ball at loop 10 (0.2 s) + 0.8 s starve = about loop 50
        assertTrue(pumpsAt >= 49 && pumpsAt <= 52, "pumped at loop " + pumpsAt);
    }

    @Test
    void intakeStowedShakesInstead() {
        AgitationPlanner p = planner();
        double t = 0; AgitationPlanner.Action got = AgitationPlanner.Action.NONE;
        for (int i = 0; i < 60 && got == AgitationPlanner.Action.NONE; i++, t += DT)
            got = p.update(t, true, true, true, 0, 0, false, true, false);
        assertEquals(AgitationPlanner.Action.SHAKE, got);
    }

    @Test
    void emptyHopperStopsAfterThreeTries() {
        AgitationPlanner p = planner();
        double t = 0; int pumps = 0; boolean pumping = false; double pumpEnd = 0;
        for (int i = 0; i < 1000; i++, t += DT) {
            if (pumping && t >= pumpEnd) pumping = false;
            AgitationPlanner.Action a = p.update(t, true, true, true, 0, 0, true, true, pumping);
            if (a == AgitationPlanner.Action.PUMP) { pumps++; pumping = true; pumpEnd = t + 1.0; }
        }
        assertEquals(3, pumps);
        assertTrue(p.status().contains("EMPTY"), p.status());
    }

    @Test
    void ballAfterPumpResetsTheCount() {
        AgitationPlanner p = planner();
        double t = 0; int pumps = 0; int shots = 0; boolean pumping = false; double pumpEnd = 0;
        for (int i = 0; i < 2000; i++, t += DT) {
            if (pumping && t >= pumpEnd) { pumping = false; shots++; } // each pump frees a ball
            AgitationPlanner.Action a = p.update(t, true, true, true, shots, 0, true, true, pumping);
            if (a == AgitationPlanner.Action.PUMP) { pumps++; pumping = true; pumpEnd = t + 1.0; }
        }
        assertTrue(pumps > 3, "kept helping while pumps keep freeing balls: " + pumps);
    }

    @Test
    void repeatedJamsTriggerAPump() {
        AgitationPlanner p = planner();
        double t = 0; int shots = 0; int jams = 0; AgitationPlanner.Action got = AgitationPlanner.Action.NONE;
        for (int i = 0; i < 100 && got == AgitationPlanner.Action.NONE; i++, t += DT) {
            if (i % 5 == 0) shots++;           // balls still trickling (not starving)
            if (i == 10 || i == 30 || i == 50) jams++;
            got = p.update(t, true, true, true, shots, jams, true, true, false);
        }
        assertEquals(AgitationPlanner.Action.PUMP, got);
    }

    @Test
    void offSwitchAndDriverOwnershipAreRespected() {
        AgitationPlanner p = planner();
        double t = 0;
        for (int i = 0; i < 100; i++, t += DT)
            assertEquals(AgitationPlanner.Action.NONE, p.update(t, false, true, true, 0, 0, true, true, false));
        AgitationPlanner q = planner();
        AgitationPlanner.Action got = AgitationPlanner.Action.NONE;
        for (int i = 0; i < 60 && got == AgitationPlanner.Action.NONE; i++, t += DT)
            got = q.update(t, true, true, true, 0, 0, true, false, false); // driver moving the intake
        assertEquals(AgitationPlanner.Action.SHAKE, got);
    }

    @Test
    void volleySummary() {
        AgitationPlanner p = planner();
        double t = 0; int shots = 0;
        for (int i = 0; i < 100; i++, t += DT) { if (i % 10 == 5) shots++; p.update(t, true, true, true, shots, 0, true, true, false); }
        for (int i = 0; i < 40; i++, t += DT) p.update(t, true, false, true, shots, 0, true, true, false);
        assertTrue(p.lastVolley().startsWith("10 balls in 2.0 s"), p.lastVolley());
    }
}
