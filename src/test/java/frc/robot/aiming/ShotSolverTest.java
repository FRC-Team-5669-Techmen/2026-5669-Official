package frc.robot.aiming;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

class ShotSolverTest {
    private static final double TOL = 1e-6;
    private static final Translation2d HUB = new Translation2d(4.6255, 4.0346);

    // Turret at robot center, HOME pointing straight out the robot's front, 270 deg range, 10 deg margin.
    private static final ShotSolver.Geometry FRONT_HOME =
        new ShotSolver.Geometry(new Translation2d(), 0.0, 270.0, 10.0);

    private static ShotSolver.Solution solve(Pose2d robot, ShotSolver.Geometry g) {
        return ShotSolver.solve(robot, 0, 0, HUB, g, d -> 1.0, false);
    }

    @Test
    void distanceIsStraightLineToHub() {
        Pose2d robot = new Pose2d(1.6255, 4.0346, Rotation2d.kZero); // 3 m left of the hub
        assertEquals(3.0, solve(robot, FRONT_HOME).distanceMeters(), TOL);
    }

    @Test
    void hubToTheRightNeedsClockwiseTurret() {
        // Robot faces +y (90 deg). Hub is at +x = to the robot's RIGHT = 90 deg clockwise of HOME.
        Pose2d robot = new Pose2d(1.6255, 4.0346, Rotation2d.fromDegrees(90));
        ShotSolver.Solution s = solve(robot, FRONT_HOME);
        assertTrue(s.reachable());
        assertEquals(90.0, s.turretDegrees(), 1e-9);
    }

    @Test
    void hubBehindTheWiresIsUnreachableAndRobotTurnsTheShortWay() {
        // Robot faces -y (-90). Hub at +x is to the robot's LEFT = 270 deg clockwise from HOME, past
        // the 260 deg aim limit (dead zone). Nearest end is 260: the robot turns 10 deg clockwise.
        Pose2d robot = new Pose2d(1.6255, 4.0346, Rotation2d.fromDegrees(-90));
        ShotSolver.Solution s = solve(robot, FRONT_HOME);
        assertFalse(s.reachable());
        assertEquals(260.0, s.turretDegrees(), 1e-9);
        assertEquals(Math.toRadians(-10.0), s.headingErrorRad(), 1e-9);

        // After turning that much, it must be reachable.
        Pose2d turned = new Pose2d(robot.getTranslation(), Rotation2d.fromDegrees(-100));
        assertTrue(solve(turned, FRONT_HOME).reachable());
    }

    @Test
    void deadZoneMiddlePicksCloserEnd() {
        // Robot faces the hub, so the turret would need 0 deg: below the 10 deg margin (dead zone).
        Pose2d robot = new Pose2d(1.6255, 4.0346, Rotation2d.kZero);
        ShotSolver.Solution s = solve(robot, FRONT_HOME);
        assertFalse(s.reachable());
        assertEquals(10.0, s.turretDegrees(), 1e-9);
        // The needed angle must grow from 0 to 10: the robot turns 10 deg counter-clockwise.
        assertEquals(Math.toRadians(10.0), s.headingErrorRad(), 1e-9);
    }

    @Test
    void homeYawOffsetShiftsTurretAngle() {
        // HOME points out the robot's LEFT (90). Robot faces +x, hub straight ahead (0 relative):
        // shooter must turn from 90 to 0 = 90 deg clockwise.
        ShotSolver.Geometry leftHome = new ShotSolver.Geometry(new Translation2d(), 90.0, 270.0, 10.0);
        Pose2d robot = new Pose2d(1.6255, 4.0346, Rotation2d.kZero);
        ShotSolver.Solution s = solve(robot, leftHome);
        assertTrue(s.reachable());
        assertEquals(90.0, s.turretDegrees(), 1e-9);
    }

    @Test
    void pivotOffsetIsMeasuredFromThePivot() {
        // Pivot 0.5 m behind center. Robot faces the hub from 3 m: shot distance is 3.5 m.
        ShotSolver.Geometry g = new ShotSolver.Geometry(new Translation2d(-0.5, 0), 0.0, 270.0, 10.0);
        Pose2d robot = new Pose2d(1.6255, 4.0346, Rotation2d.fromDegrees(90)); // pivot then at y-0.5 rotated
        ShotSolver.Solution s = solve(robot, g);
        Translation2d pivot = new Translation2d(1.6255, 4.0346 - 0.5);
        assertEquals(HUB.getDistance(pivot), s.distanceMeters(), TOL);
    }

    @Test
    void shootingWhileMovingAimsBehindTheMotion() {
        // Robot 3 m left of the hub, driving +y at 2 m/s, flight time 0.5 s: aim 1 m toward -y.
        Pose2d robot = new Pose2d(1.6255, 4.0346, Rotation2d.kZero);
        ShotSolver.Solution s = ShotSolver.solve(robot, 0, 2.0, HUB, FRONT_HOME, d -> 0.5, true);
        assertEquals(HUB.getX(), s.aimPoint().getX(), TOL);
        assertEquals(HUB.getY() - 1.0, s.aimPoint().getY(), TOL);
        assertEquals(Math.hypot(3.0, 1.0), s.distanceMeters(), TOL);
    }

    @Test
    void cameraTurnsWithTheTurret() {
        // Pivot at origin, camera 0.2 m ahead of it. HOME = front. Turret 90 clockwise -> camera
        // points out the RIGHT: 0.2 m to the right (left = -0.2), yaw -90.
        double[] c = ShotSolver.cameraInRobot(new Translation2d(), 0.0, 90.0, 0.2, 0.0);
        assertEquals(0.0, c[0], TOL);
        assertEquals(-0.2, c[1], TOL);
        assertEquals(-90.0, c[2], TOL);
    }

    @Test
    void wrapping() {
        assertEquals(350.0, ShotSolver.wrap360(-10), TOL);
        assertEquals(-170.0, ShotSolver.wrap180(190), TOL);
        assertEquals(-180.0, ShotSolver.wrap180(180), TOL);
    }

    // ---------------- shoot on the move, full model ----------------

    @Test
    void spinningInPlaceFeedsTheTurretTheOppositeWay() {
        // Robot spins CCW at 2 rad/s, turret at center: the turret must turn clockwise at 2 rad/s.
        Pose2d robot = new Pose2d(1.6255, 4.0346, Rotation2d.fromDegrees(90));
        ShotSolver.Solution s = ShotSolver.solve(robot, 0, 0, 2.0, HUB, FRONT_HOME, d -> 0.5, 0.0);
        assertEquals(Math.toDegrees(2.0), s.turretRateDegPerSec(), 1e-6);
    }

    @Test
    void drivingPastTheTargetTurnsTheTurret() {
        // Robot 3 m from the hub on its -x side, facing +y (hub on its right, turret 90).
        // Driving +y at 1 m/s with no flight time: the hub slides behind it (turret angle grows
        // toward 180), so the turret turns clockwise at v / r = 1/3 rad/s.
        Pose2d robot = new Pose2d(1.6255, 4.0346, Rotation2d.fromDegrees(90));
        ShotSolver.Solution s = ShotSolver.solve(robot, 0, 1.0, 0.0, HUB, FRONT_HOME, d -> 0.0, 0.0);
        assertEquals(Math.toDegrees(1.0 / 3.0), s.turretRateDegPerSec(), 1e-6);
    }

    @Test
    void spinningOffCenterTurretLeadsForItsOwnVelocity() {
        // Pivot 0.5 m left of center, robot faces +x, spins CCW at 2 rad/s: the pivot moves at
        // 1 m/s toward -x (straight back), i.e. AWAY from a hub ahead of it -> no sideways lead,
        // but the virtual target moves 1 m/s * tof TOWARD the robot... aim = hub + (1,0)*tof.
        ShotSolver.Geometry g = new ShotSolver.Geometry(new Translation2d(0, 0.5), 0.0, 360.0, 0.0);
        Pose2d robot = new Pose2d(1.6255, 3.5346, Rotation2d.kZero); // pivot at hub y
        ShotSolver.Solution s = ShotSolver.solve(robot, 0, 0, 2.0, HUB, g, d -> 0.5, 0.0);
        assertEquals(HUB.getX() + 0.5, s.aimPoint().getX(), 1e-6);
        assertEquals(HUB.getY(), s.aimPoint().getY(), 1e-6);
    }

    @Test
    void releaseLatencyAimsFromWhereTheRobotWillBe() {
        // Driving straight at the hub at 2 m/s from 3 m with 0.25 s latency: the ball leaves 0.5 m
        // closer. Flight time 0 isolates the latency effect.
        Pose2d robot = new Pose2d(1.6255, 4.0346, Rotation2d.fromDegrees(90));
        ShotSolver.Solution s = ShotSolver.solve(robot, 2.0, 0, 0.0, HUB, FRONT_HOME, d -> 0.0, 0.25);
        assertEquals(2.5, s.distanceMeters(), 1e-6);
    }

    @Test
    void distanceDependentFlightTimeConverges() {
        // TOF grows with distance (0.3 s per m): the virtual point must satisfy aim = hub - v*tof(aim).
        Pose2d robot = new Pose2d(1.6255, 4.0346, Rotation2d.fromDegrees(90));
        java.util.function.DoubleUnaryOperator tof = d -> 0.3 * d;
        ShotSolver.Solution s = ShotSolver.solve(robot, 0, 1.5, 0.0, HUB, FRONT_HOME, tof, 0.0);
        double expectedY = HUB.getY() - 1.5 * tof.applyAsDouble(s.distanceMeters());
        assertEquals(expectedY, s.aimPoint().getY(), 0.01);
    }
}
