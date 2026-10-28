package frc.robot.characterization;

/**
 * WHEEL RADIUS from spinning in place (the method 6328 publishes in its AdvantageKit template).
 * Spinning in place, every wheel rolls along a circle of radius R (the drive base radius, center
 * to module). The gyro says how far the robot really turned; the wheels say how far they think
 * they rolled using the radius in the code. The ratio corrects the radius:
 * <pre>  true radius = code radius x (gyro turn x R) / (distance the wheels think they rolled)</pre>
 * A worn tread is smaller than the catalog 2 in, so the robot thinks it went further than it did:
 * every path ends short by the same percent.
 *
 * <p>Pure math, no hardware: tested in CharacterizationTest.
 */
public final class WheelRadiusCalc {
    private WheelRadiusCalc() {}

    /**
     * @param codeRadius      wheel radius the odometry uses now (m)
     * @param driveBaseRadius robot center to a module (m)
     * @param gyroTurnRad     how far the gyro says the robot turned (rad, any sign)
     * @param wheelDistances  how far each wheel thinks it rolled (m, any sign)
     * @return the measured wheel radius (m), or NaN if the robot barely turned
     */
    public static double radius(double codeRadius, double driveBaseRadius, double gyroTurnRad, double[] wheelDistances) {
        double avg = 0;
        for (double d : wheelDistances) avg += Math.abs(d);
        avg /= wheelDistances.length;
        if (Math.abs(gyroTurnRad) < Math.toRadians(90) || avg < 1e-6) return Double.NaN;
        return codeRadius * Math.abs(gyroTurnRad) * driveBaseRadius / avg;
    }
}
