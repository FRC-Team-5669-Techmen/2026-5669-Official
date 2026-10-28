package frc.robot.characterization;

/**
 * SLIP CURRENT (CTRE's procedure for the swerve "slip current"): with the robot's bumper flat
 * against a wall, the drive voltage climbs slowly. The wheels push harder and harder without
 * turning, until the tread breaks loose and they spin. The drive motor current just before they
 * spin is the most the carpet can take: any current above it only spins the wheels (lost
 * traction, and odometry that thinks the robot moved).
 *
 * <p>Pure logic, no hardware: tested in CharacterizationTest.
 */
public class SlipDetector {
    private final double spinSpeed;
    private double peakAmps = 0;
    private double slipAmps = Double.NaN;

    /** @param spinSpeed wheel speed (m/s) that means "the wheels broke loose" (the robot can't move) */
    public SlipDetector(double spinSpeed) { this.spinSpeed = spinSpeed; }

    /**
     * @param avgAmps      average drive motor stator current
     * @param avgWheelSpeed average wheel surface speed (m/s)
     * @return true once the wheels have slipped (result ready)
     */
    public boolean update(double avgAmps, double avgWheelSpeed) {
        if (!Double.isNaN(slipAmps)) return true;
        if (Math.abs(avgWheelSpeed) >= spinSpeed) {
            slipAmps = peakAmps;
            return true;
        }
        peakAmps = Math.max(peakAmps, Math.abs(avgAmps));
        return false;
    }

    public double slipAmps() { return slipAmps; }
    public double peakAmps() { return peakAmps; }
}
