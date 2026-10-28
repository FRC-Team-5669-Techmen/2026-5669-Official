package frc.robot.power;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

class BatteryEstimatorTest {
    @Test
    void findsResistanceFromVaryingLoad() {
        BatteryEstimator e = new BatteryEstimator();
        Random rnd = new Random(1);
        double voc = 12.6, r = 0.022;
        for (int k = 0; k < 500; k++) {          // 10 s at 50 Hz
            double t = k * 0.02;
            double amps = 20 + 100 * Math.abs(Math.sin(t * 2));   // 20 to 120 A
            double v = voc - amps * r + rnd.nextGaussian() * 0.03;
            e.add(t, v, amps);
        }
        assertTrue(e.fits() > 5);
        assertEquals(0.022, e.resistance(), 0.002);
        assertEquals(12.6, e.openCircuitVolts(), 0.1);
    }

    @Test
    void noFitWithoutCurrentChange() {
        BatteryEstimator e = new BatteryEstimator();
        for (int k = 0; k < 200; k++) e.add(k * 0.02, 12.3, 10 + (k % 2));
        assertEquals(0, e.fits());
        assertTrue(Double.isNaN(e.resistance()));
    }

    @Test
    void verdicts() {
        assertEquals("OK", BatteryEstimator.verdict(12.7, 0.020, 9.5, 150, 0, 0.030, 12.4));
        assertTrue(BatteryEstimator.verdict(12.2, 0.020, 9, 100, 0, 0.030, 12.4).startsWith("CHARGE"));
        assertTrue(BatteryEstimator.verdict(12.7, 0.045, 6, 140, 1, 0.030, 12.4).contains("HIGH RESISTANCE"));
        assertTrue(BatteryEstimator.verdict(12.7, 0.020, 6, 300, 1, 0.030, 12.4).startsWith("ROBOT DRAW"));
    }
}
