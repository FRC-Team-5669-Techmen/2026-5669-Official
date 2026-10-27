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

    // ---------------- JamDetector lockout ----------------

    @Test
    void threeJamsInARowPauseAutoReverse() {
        JamDetector j = new JamDetector(45, 8, 0.15, 0.3, 0.25, 3.0, 3, 3.0);
        double t = 0;
        for (int i = 0; i < 25; i++, t += DT) j.update(t, 20, 40);
        for (int k = 0; k < 6; k++) {               // keeps stalling
            for (int i = 0; i < 40; i++, t += DT) j.update(t, 70, 1);
        }
        assertEquals(3, j.jams(), "after 3 quick jams it stops hammering back and forth");
    }

    // ---------------- AgitationPlanner ----------------

    static AgitationPlanner planner() { return new AgitationPlanner(new AgitationPlanner.Params()); }

    static AgitationPlanner.Inputs in(int shots, boolean pumpOk, boolean shimmyOk, boolean busy) {
        AgitationPlanner.Inputs in = new AgitationPlanner.Inputs();
        in.enabled = true; in.feeding = true; in.shooterUp = true;
        in.shots = shots; in.jams = 0;
        in.pumpPossible = pumpOk; in.shimmyPossible = shimmyOk; in.busy = busy;
        return in;
    }

    @Test
    void steadyFlowNeverAgitates() {
        AgitationPlanner p = planner();
        double t = 0; int shots = 0;
        for (int i = 0; i < 250; i++, t += DT) {
            if (i % 12 == 0) shots++; // a ball every 0.24 s
            assertEquals(AgitationPlanner.Action.NONE, p.update(t, in(shots, true, true, false)));
        }
    }

    @Test
    void gapLadderNudgesThenPumps() {
        AgitationPlanner p = planner();
        double t = 0; int nudgeAt = -1, pumpAt = -1;
        for (int i = 0; i < 100; i++, t += DT) {
            AgitationPlanner.Action a = p.update(t, in(1, true, false, false));
            if (a == AgitationPlanner.Action.NUDGE && nudgeAt < 0) nudgeAt = i;
            if (a == AgitationPlanner.Action.PUMP) { pumpAt = i; break; }
        }
        // ball at loop 0; nudge at ~0.35 s (loop 18), pump at ~0.8 s (loop 41)
        assertTrue(nudgeAt >= 17 && nudgeAt <= 19, "nudge at " + nudgeAt);
        assertTrue(pumpAt >= 40 && pumpAt <= 42, "pump at " + pumpAt);
    }

    @Test
    void ballAfterNudgeResetsTheLadder() {
        AgitationPlanner p = planner();
        double t = 0; int shots = 0; boolean pumped = false;
        for (int i = 0; i < 300; i++, t += DT) {
            AgitationPlanner.Action a = p.update(t, in(shots, true, false, false));
            if (a == AgitationPlanner.Action.NUDGE) shots++;   // every nudge frees a ball
            pumped |= a == AgitationPlanner.Action.PUMP;
        }
        assertFalse(pumped, "nudges that work never escalate to a pump");
    }

    @Test
    void shimmyComesAfterAPumpThatDidNotWork() {
        AgitationPlanner p = planner();
        double t = 0; boolean busy = false; double busyEnd = 0;
        java.util.List<AgitationPlanner.Action> big = new java.util.ArrayList<>();
        for (int i = 0; i < 1000; i++, t += DT) {
            if (busy && t >= busyEnd) busy = false;
            AgitationPlanner.Action a = p.update(t, in(0, true, true, busy));
            if (a == AgitationPlanner.Action.PUMP || a == AgitationPlanner.Action.SHIMMY) {
                big.add(a); busy = true; busyEnd = t + (a == AgitationPlanner.Action.PUMP ? 1.0 : 0.4);
            }
        }
        assertEquals(java.util.List.of(AgitationPlanner.Action.PUMP, AgitationPlanner.Action.SHIMMY,
            AgitationPlanner.Action.SHIMMY), big);
        assertTrue(p.status().contains("EMPTY"), p.status());
    }

    @Test
    void intakeUnavailableShakesInstead() {
        AgitationPlanner p = planner();
        double t = 0; AgitationPlanner.Action big = AgitationPlanner.Action.NONE;
        for (int i = 0; i < 100; i++, t += DT) {
            AgitationPlanner.Action a = p.update(t, in(0, false, false, false));
            if (a == AgitationPlanner.Action.SHAKE) { big = a; break; }
        }
        assertEquals(AgitationPlanner.Action.SHAKE, big);
    }

    @Test
    void emptyHopperStopsAfterThreeTries() {
        AgitationPlanner p = planner();
        double t = 0; int big = 0; boolean busy = false; double busyEnd = 0;
        for (int i = 0; i < 1500; i++, t += DT) {
            if (busy && t >= busyEnd) busy = false;
            AgitationPlanner.Action a = p.update(t, in(0, true, false, busy));
            if (a == AgitationPlanner.Action.PUMP) { big++; busy = true; busyEnd = t + 1.0; }
        }
        assertEquals(3, big);
        assertTrue(p.status().contains("EMPTY"), p.status());
    }

    @Test
    void repeatedJamsTriggerAPump() {
        AgitationPlanner p = planner();
        double t = 0; int shots = 0; AgitationPlanner.Action got = AgitationPlanner.Action.NONE;
        for (int i = 0; i < 100 && got != AgitationPlanner.Action.PUMP; i++, t += DT) {
            if (i % 5 == 0) shots++;           // balls still trickling (not starving)
            AgitationPlanner.Inputs x = in(shots, true, false, false);
            x.jams = i >= 50 ? 3 : i >= 30 ? 2 : i >= 10 ? 1 : 0;
            got = p.update(t, x);
        }
        assertEquals(AgitationPlanner.Action.PUMP, got);
    }

    @Test
    void offSwitchIsRespected() {
        AgitationPlanner p = planner();
        double t = 0;
        for (int i = 0; i < 100; i++, t += DT) {
            AgitationPlanner.Inputs x = in(0, true, true, false);
            x.enabled = false;
            assertEquals(AgitationPlanner.Action.NONE, p.update(t, x));
        }
    }

    @Test
    void volleySummary() {
        AgitationPlanner p = planner();
        double t = 0; int shots = 0;
        for (int i = 0; i < 100; i++, t += DT) { if (i % 10 == 5) shots++; p.update(t, in(shots, true, false, false)); }
        for (int i = 0; i < 40; i++, t += DT) { AgitationPlanner.Inputs x = in(shots, true, false, false); x.feeding = false; p.update(t, x); }
        assertTrue(p.lastVolley().startsWith("10 balls in 2.0 s"), p.lastVolley());
    }
}
