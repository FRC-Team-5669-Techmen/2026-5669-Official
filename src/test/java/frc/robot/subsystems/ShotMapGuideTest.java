package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;

import org.junit.jupiter.api.Test;

class ShotMapGuideTest {
    @Test
    void biggestGapFindsTheWidestHoleInTheTable() {
        // Oct 28 table: the widest hole is 3.13 to 4.73 m (1.60 m; 1.69 to 3.13 is 1.44 m).
        assertEquals("3.13 to 4.73 m", ShotMapGuide.biggestGap(List.of(1.69, 3.13, 4.73, 5.33)));
        // An empty table: the whole 1.5 to 5.5 m range.
        assertEquals("1.50 to 5.50 m", ShotMapGuide.biggestGap(List.of()));
        // Points past the range don't stretch it.
        assertEquals("1.50 to 3.00 m", ShotMapGuide.biggestGap(List.of(0.8, 3.0, 4.2, 6.5)));
    }

    @Test
    void shotLogHeaderHasOneNameForEveryColumn() {
        assertEquals(23, ShotMapGuide.kHeader.split(",").length);
    }
}
