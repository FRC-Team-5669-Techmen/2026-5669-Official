package frc.robot.practice;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;
import java.util.Map;

import org.junit.jupiter.api.Test;

class MatchTelemetryTest {
    private static MatchTelemetry.Sample s(double t, String phase, double volts, boolean rt, boolean ready, String why) {
        return new MatchTelemetry.Sample(t, 140 - t, phase, volts, false, 20, 1.0, true, ready, rt, ready && rt, why,
            false, false, false, true, 0.5, false, false, false, 0, 0.4);
    }

    @Test
    @SuppressWarnings("unchecked")
    void summarizesBatteryShotsReadinessAndEvents() {
        MatchTelemetry m = new MatchTelemetry(Map.of("kind", "practice"));
        for (int i = 0; i <= 100; i++) {
            double t = i * 0.05;                       // 5 s
            boolean rt = i >= 20;                      // RT held the last 4 s
            boolean ready = i >= 60;                   // ready the last 2 s
            m.sample(s(t, i < 50 ? "SHIFT 1" : "SHIFT 2", i == 70 ? 7.5 : 12.0, rt, ready, "WAITING: spinning up (3200 of 3600)"));
        }
        m.shot("SHIFT 2", "open");
        m.shot("SHIFT 2", "closed");
        m.shot("AUTO", "auto");
        m.launch(3550, 3600);
        Map<String, Object> out = m.summary();
        Map<String, Object> batt = (Map<String, Object>) out.get("battery");
        assertEquals(7.5, (double) batt.get("min"), 1e-9);
        assertEquals(12.0, (double) batt.get("start"), 1e-9);
        assertTrue(((String) batt.get("minWhere")).startsWith("SHIFT 2"));
        Map<String, Object> sc = (Map<String, Object>) out.get("scoring");
        assertEquals(1, sc.get("ballsHubOpen"));
        assertEquals(1, sc.get("ballsHubClosed"));
        assertEquals(1, sc.get("autoBalls"));
        assertEquals(3, sc.get("ballsTotal"));
        assertEquals(50.0, (double) sc.get("readyWhileRtHeldPct"), 3.0); // 2 of 4 s
        Map<String, Double> why = (Map<String, Double>) sc.get("notReadySecondsByReason");
        assertEquals(1, why.size());
        assertTrue(why.keySet().iterator().next().startsWith("WAITING: spinning up"));
        assertEquals(50.0, (double) sc.get("launchRpmErrorMean"), 1e-9);
        List<Map<String, Object>> ev = (List<Map<String, Object>>) out.get("events");
        assertTrue(ev.stream().anyMatch(e -> "phase SHIFT 2".equals(e.get("text"))));
        assertEquals(3, ((List<?>) out.get("phases")).size()); // SHIFT 1, SHIFT 2, AUTO (from the auto shot)
        assertEquals("practice", out.get("kind"));
    }

    @Test
    void reasonDropsNumbers() {
        assertEquals("WAITING: spinning up", MatchTelemetry.reason("WAITING: spinning up (3200 of 3600)"));
        assertEquals("HUB CLOSED: opens in # s", MatchTelemetry.reason("HUB CLOSED: opens in 7 s"));
    }
}
