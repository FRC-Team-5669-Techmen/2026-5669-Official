package frc.robot.aiming;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;

import org.junit.jupiter.api.Test;

class ShotTableTest {
    @Test
    void interpolatesAndClamps() {
        ShotTable t = new ShotTable();
        t.put(new ShotTable.Point(2.0, 3000, 0.5, 0.6));
        t.put(new ShotTable.Point(4.0, 5000, 1.5, 1.0));
        assertEquals(4000, t.rpm(3.0), 1e-9);
        assertEquals(1.0, t.hood(3.0), 1e-9);
        assertEquals(3000, t.rpm(0.5), 1e-9);   // below range -> nearest end
        assertEquals(5000, t.rpm(9.0), 1e-9);   // above range -> nearest end
    }

    @Test
    void recordedPointReplacesNearbyDefault() {
        ShotTable t = ShotTable.defaults();
        int before = t.points().size();
        t.put(new ShotTable.Point(3.2, 3100, 3.1, 0.7)); // within 0.25 m of the 3.13 m point (3.55 is not)
        assertEquals(before, t.points().size());
        assertEquals(3100, t.rpm(3.2), 1e-9);
    }

    @Test
    void measuredTableHoodRisesWithDistance() {
        // Further shots never need a lower hood; a point that breaks this is a typo. (RPM can dip:
        // a lower hood needs more speed, see ShotTable.defaults.)
        ShotTable.Point prev = null;
        for (ShotTable.Point p : ShotTable.defaults().points().values()) {
            if (prev != null) {
                assertTrue(p.hood() >= prev.hood(), "hood drops at " + p.distance());
            }
            assertTrue(p.hood() >= 0.2 && p.hood() <= 7.87, "hood out of travel at " + p.distance());
            prev = p;
        }
        assertTrue(ShotTable.defaults().points().size() >= 7);
    }

    @Test
    void encodeDecodeRoundTrip() {
        List<ShotTable.Point> pts = List.of(new ShotTable.Point(2.5, 3900, 0.7, 0.7),
                                            new ShotTable.Point(4.25, 4800, 1.333, 0.95));
        List<ShotTable.Point> back = ShotTable.decode(ShotTable.encode(pts));
        assertEquals(2, back.size());
        assertEquals(4.25, back.get(1).distance(), 1e-9);
        assertEquals(1.333, back.get(1).hood(), 1e-9);
        assertTrue(ShotTable.decode("junk;1:2").isEmpty());
    }
}
