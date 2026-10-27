package frc.robot.aiming;

import java.util.function.DoubleUnaryOperator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
     * @param pivotField      turret pivot position on the field (at the predicted release moment)
     * @param turretRateDegPerSec rate the turret target is changing, deg/s clockwise (turret feedforward)
     */
    public record Solution(
        double distanceMeters,
        double turretDegrees,
        boolean reachable,
        double headingErrorRad,
        double timeOfFlight,
        Translation2d aimPoint,
        Translation2d pivotField,
        double turretRateDegPerSec) {}   // how fast the turret target is moving (feedforward)

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
        return solve(robot, vxField, vyField, 0.0, hub, geo, timeOfFlight, leadForMotion ? 0.0 : -1.0);
    }

    /**
     * SHOOT ON THE MOVE, full version.
     * <ol>
     *   <li>Look ahead: predict where the robot will be when the ball actually leaves
     *       (releaseLatency seconds from now: feed travel + control delay), from its velocity and spin.</li>
     *   <li>The ball leaves with the TURRET PIVOT's velocity, not the robot center's: robot velocity
     *       plus spin x the pivot's offset (the pivot sits off-center, so spinning moves it).</li>
     *   <li>Virtual target: aim at target - pivotVelocity x flightTime. Flight time depends on the
     *       distance to that point, so iterate until it settles (under 1 cm, at most 10 rounds).
     *       The shot table is then read at the VIRTUAL distance (RPM/hood for where the ball must
     *       really travel relative to the moving launcher).</li>
     *   <li>Turret rate feedforward: how fast the turret angle to the virtual target is changing
     *       (robot sliding past it plus robot spin), so the turret tracks instead of lagging.</li>
     * </ol>
     *
     * @param omegaRadPerSec robot spin, CCW positive
     * @param target         field point to hit (hub center, or a passing landing point)
     * @param releaseLatency seconds from "now" until the ball leaves; NEGATIVE = no motion lead at all
     */
    public static Solution solve(Pose2d robot, double vxField, double vyField, double omegaRadPerSec,
                                 Translation2d target, Geometry geo, DoubleUnaryOperator timeOfFlight,
                                 double releaseLatency) {
        boolean lead = releaseLatency >= 0;
        double dt = lead ? releaseLatency : 0.0;

        // 1) Robot at the release moment.
        Rotation2d heading = robot.getRotation().plus(Rotation2d.fromRadians(omegaRadPerSec * dt));
        Translation2d center = robot.getTranslation().plus(new Translation2d(vxField * dt, vyField * dt));
        Translation2d offset = geo.turretPivot().rotateBy(heading);       // pivot offset, field frame
        Translation2d pivot = center.plus(offset);

        // 2) Pivot velocity = robot velocity + spin x offset.
        double pvx = lead ? vxField - omegaRadPerSec * offset.getY() : 0.0;
        double pvy = lead ? vyField + omegaRadPerSec * offset.getX() : 0.0;

        // 3) Virtual target, iterated.
        Translation2d aim = target;
        double tof = timeOfFlight.applyAsDouble(target.getDistance(pivot));
        if (lead && (pvx != 0 || pvy != 0)) {
            for (int i = 0; i < 10; i++) {
                Translation2d next = new Translation2d(target.getX() - pvx * tof, target.getY() - pvy * tof);
                double moved = next.getDistance(aim);
                aim = next;
                tof = timeOfFlight.applyAsDouble(aim.getDistance(pivot));
                if (moved < 0.01) break;
            }
        }
        double distance = aim.getDistance(pivot);

        // 4) Turret rate: the line pivot->aim turns as the pivot moves (aim point is fixed for a
        //    steady velocity). Field angle rate = cross(r, -v) / |r|^2, CCW.
        Translation2d r = aim.minus(pivot);
        double r2 = Math.max(r.getX() * r.getX() + r.getY() * r.getY(), 1e-6);
        double fieldRate = (r.getX() * (-pvy) - r.getY() * (-pvx)) / r2;          // rad/s CCW
        // turret (clockwise from HOME) = home - (fieldAngle - heading): rate = -(fieldRate - omega)
        double turretRate = Math.toDegrees(-(fieldRate - omegaRadPerSec));

        double fieldAngleDeg = Math.toDegrees(Math.atan2(r.getY(), r.getX()));
        double relativeDeg = fieldAngleDeg - heading.getDegrees(); // CCW from robot front
        // Turret counts clockwise from HOME: shooter direction = home - turret.
        double turretDeg = wrap360(geo.homeYawDegrees() - relativeDeg);

        double lo = geo.marginDegrees();
        double hi = geo.rangeDegrees() - geo.marginDegrees();
        if (turretDeg >= lo && turretDeg <= hi) {
            return new Solution(distance, turretDeg, true, 0.0, tof, aim, pivot, turretRate);
        }

        // In the dead zone: aim at the closer end and say how far the robot must turn.
        // Turning the robot CCW by d makes the needed turret angle grow by d.
        double toLo = wrap180(lo - turretDeg);   // robot turn that puts the target at the low end
        double toHi = wrap180(hi - turretDeg);   // ... at the high end
        boolean useLo = Math.abs(toLo) <= Math.abs(toHi);
        double edge = useLo ? lo : hi;
        double robotTurnDeg = useLo ? toLo : toHi;
        return new Solution(distance, edge, false, Math.toRadians(robotTurnDeg), tof, aim, pivot, 0.0);
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
