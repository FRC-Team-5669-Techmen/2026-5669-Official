package frc.robot.aiming;

import java.util.function.DoubleUnaryOperator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Pure aiming math, no hardware: where the turret must point and how far the shot is, from the
 * robot's field position. Unit-tested in ShotSolverTest.
 *
 * <p>Angles: field angles are counter-clockwise from the field +x axis (WPILib). The turret
 * angle is degrees CLOCKWISE from HOME (that is how Goober counts: positive = turning RIGHT).
 */
public final class ShotSolver {
    private ShotSolver() {}

    /** Robot geometry the solver needs. */
    public record Geometry(
        Translation2d turretPivot,   // from robot center, robot frame (x forward, y left), meters
        double homeYawDegrees,       // shooter direction at HOME, CCW from robot forward
        double rangeDegrees,         // turret travel from HOME (about 270)
        double marginDegrees) {}     // stay this far inside each end

    /**
     * @param distanceMeters  from the turret pivot to the point being aimed at (for the shot table)
     * @param turretDegrees   turret target, degrees clockwise from HOME, always inside the aim range
     * @param reachable       false when the hub is in the dead zone; turretDegrees is then the nearest end
     * @param headingErrorRad how far the ROBOT must turn (CCW positive) to bring the hub into reach; 0 if reachable
     * @param timeOfFlight    seconds, from the shot table, for the distance
     * @param aimPoint        field point actually aimed at (the hub, shifted for robot motion)
     * @param pivotField      turret pivot position on the field
     */
    public record Solution(
        double distanceMeters,
        double turretDegrees,
        boolean reachable,
        double headingErrorRad,
        double timeOfFlight,
        Translation2d aimPoint,
        Translation2d pivotField) {}

    /**
     * @param robot          robot pose on the field
     * @param vxField        robot velocity on the field, m/s (blue-origin x)
     * @param vyField        robot velocity on the field, m/s (blue-origin y)
     * @param hub            hub center on the field
     * @param geo            turret geometry
     * @param timeOfFlight   distance (m) -> ball flight time (s)
     * @param leadForMotion  aim ahead for robot motion (shooting while moving)
     */
    public static Solution solve(Pose2d robot, double vxField, double vyField, Translation2d hub,
                                 Geometry geo, DoubleUnaryOperator timeOfFlight, boolean leadForMotion) {
        Translation2d pivot = robot.getTranslation().plus(geo.turretPivot().rotateBy(robot.getRotation()));

        // The ball keeps the robot's velocity, so aim at hub - velocity * flight time.
        // Flight time depends on distance, which depends on the aim point: 4 rounds converge.
        Translation2d aim = hub;
        double tof = timeOfFlight.applyAsDouble(hub.getDistance(pivot));
        if (leadForMotion) {
            for (int i = 0; i < 4; i++) {
                aim = new Translation2d(hub.getX() - vxField * tof, hub.getY() - vyField * tof);
                tof = timeOfFlight.applyAsDouble(aim.getDistance(pivot));
            }
        }
        double distance = aim.getDistance(pivot);

        Translation2d toAim = aim.minus(pivot);
        double fieldAngleDeg = Math.toDegrees(Math.atan2(toAim.getY(), toAim.getX()));
        double relativeDeg = fieldAngleDeg - robot.getRotation().getDegrees(); // CCW from robot front
        // Turret counts clockwise from HOME: shooter direction = home - turret.
        double turretDeg = wrap360(geo.homeYawDegrees() - relativeDeg);

        double lo = geo.marginDegrees();
        double hi = geo.rangeDegrees() - geo.marginDegrees();
        if (turretDeg >= lo && turretDeg <= hi) {
            return new Solution(distance, turretDeg, true, 0.0, tof, aim, pivot);
        }

        // In the dead zone: aim at the closer end and say how far the robot must turn.
        // Turning the robot CCW by d makes the needed turret angle grow by d.
        double toLo = wrap180(lo - turretDeg);   // robot turn that puts the hub at the low end
        double toHi = wrap180(hi - turretDeg);   // ... at the high end
        boolean useLo = Math.abs(toLo) <= Math.abs(toHi);
        double edge = useLo ? lo : hi;
        double robotTurnDeg = useLo ? toLo : toHi;
        return new Solution(distance, edge, false, Math.toRadians(robotTurnDeg), tof, aim, pivot);
    }

    /** Wraps to [0, 360). */
    public static double wrap360(double deg) {
        double r = deg % 360.0;
        return r < 0 ? r + 360.0 : r;
    }

    /** Wraps to [-180, 180). */
    public static double wrap180(double deg) {
        return wrap360(deg + 180.0) - 180.0;
    }

    /**
     * Where the Limelight is in ROBOT space for a given turret angle, for
     * LimelightHelpers.setCameraPose_RobotSpace. Returns {forward, left, yawDegCCW}.
     *
     * @param turretDegrees  clockwise from HOME
     * @param camForward     lens ahead of the pivot along the shooter direction
     * @param camLeft        lens left of the shooter direction
     */
    public static double[] cameraInRobot(Translation2d pivot, double homeYawDeg, double turretDegrees,
                                         double camForward, double camLeft) {
        double yawDeg = homeYawDeg - turretDegrees;
        double yaw = Math.toRadians(yawDeg);
        double fwd = pivot.getX() + camForward * Math.cos(yaw) - camLeft * Math.sin(yaw);
        double left = pivot.getY() + camForward * Math.sin(yaw) + camLeft * Math.cos(yaw);
        return new double[] {fwd, left, wrap180(yawDeg)};
    }
}
