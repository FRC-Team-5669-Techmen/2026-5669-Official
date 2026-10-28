package frc.robot.characterization;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class CharacterizationTest {
    @Test
    void wornTreadReadsSmaller() {
        // Code says 2.000 in; the real wheel is 1.950 in. Spin 3 turns: each wheel really rolls
        // 3 x 2 pi x R, but the code (using 2.000 in) thinks it rolled 2.000/1.950 times further.
        double code = 2.000 * 0.0254, real = 1.950 * 0.0254, R = 0.3976;
        double turn = 3 * 2 * Math.PI;
        double thinks = turn * R * code / real;
        double r = WheelRadiusCalc.radius(code, R, turn, new double[] {thinks, -thinks, thinks, thinks});
        assertEquals(real, r, 1e-9);
        assertTrue(Double.isNaN(WheelRadiusCalc.radius(code, R, 0.5, new double[] {0.2, 0.2, 0.2, 0.2})));
    }

    @Test
    void feedforwardFitFindsKsAndKv() {
        FeedforwardFit f = new FeedforwardFit(0.05);
        for (double v = 0; v < 3.0; v += 0.02) {
            double speed = Math.max(0, (v - 0.18) / 2.6);          // kS 0.18 V, kV 2.6 V per m/s
            f.add(v, speed);
        }
        assertEquals(2.6, f.kV(), 1e-6);
        assertEquals(0.18, f.kS(), 1e-6);
        // 2.6 V per m/s on a 2 in wheel through 7.03:1 = ~0.118 V per motor rot/s
        assertEquals(0.118, FeedforwardFit.perMotorRps(2.6, 0.0508, 7.03125), 0.001);
    }

    @Test
    void slipCurrentIsThePeakBeforeTheWheelsSpin() {
        SlipDetector s = new SlipDetector(0.3);
        double amps = 0;
        for (int i = 0; i < 100; i++) { amps += 0.7; assertFalse(s.update(amps, 0.01)); }  // pushing the wall
        assertTrue(s.update(40, 0.8));                                                       // broke loose
        assertEquals(70.0, s.slipAmps(), 0.5);
    }
}
