package frc.robot.defense;

/**
 * G420 TOWER protection: in the last 30 s, touching an opponent robot that touches THEIR tower is
 * a MAJOR FOUL (and can hand them LEVEL 3 points), no matter who started the contact. This says
 * when our robot is close enough to their tower that contact is likely.
 *
 * <p>Towers (2026 welded field, WPILib blue-origin meters): 1.251 m wide, base 1.148 m deep, on the
 * alliance wall. Blue tower centered at y 3.745 (tag 31), red at y 4.324 (tag 15): the field is
 * rotated, not mirrored.
 *
 * <p>Pure math, no hardware: tested in DefenseTest.
 */
public final class TowerGuard {
    public static final double kFieldLength = 16.541;
    public static final double kBlueTowerY = 3.745;
    public static final double kRedTowerY = 4.324;
    public static final double kTowerHalfWidth = 1.251 / 2;
    public static final double kTowerDepth = 1.148;

    private TowerGuard() {}

    /**
     * @param x, y      robot center (field meters)
     * @param weAreRed  our alliance (the guarded tower is the OTHER one)
     * @param margin    robot half size plus room for a robot climbing it (m)
     */
    public static boolean nearTheirTower(double x, double y, boolean weAreRed, double margin) {
        double depthFromTheirWall = weAreRed ? x : kFieldLength - x; // red's opponent wall is blue's, x = 0
        double towerY = weAreRed ? kBlueTowerY : kRedTowerY;
        return depthFromTheirWall <= kTowerDepth + margin && Math.abs(y - towerY) <= kTowerHalfWidth + margin;
    }

    /** Last 30 s of the match (teleop match time counts down; endgame 0:30). */
    public static boolean endgame(double matchTime) {
        return matchTime > 0 && matchTime <= 30.0;
    }
}
