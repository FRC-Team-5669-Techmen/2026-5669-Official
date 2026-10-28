package frc.robot.subsystems;

import java.util.Locale;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * BACKUP SHOTS: three known spots the robot can shoot from even with the camera dead.
 * Park on the spot, press its button (Operator A / X / LB): the robot puts itself on that spot on
 * its field map (keeping its gyro heading), turns on auto-aim, and aims/spins/sets the hood from the
 * shot table. Then hold RT like always.
 *
 * <ul>
 *   <li>MIDSHOT (Operator A): right between our hub and our climb.</li>
 *   <li>LEFT TRENCH (Operator X): poking out of the trench on the LEFT as seen from our driver station.</li>
 *   <li>RIGHT TRENCH (Operator LB): poking out of the RIGHT trench.</li>
 * </ul>
 *
 * <p>Spots are stored relative to OUR hub ("forward" = toward our driver station wall, "left" =
 * driver's left), so they work for red and blue. Defaults are estimates from the official field;
 * on Shot Map Day park on each spot with the camera working and press "Save Spot: ..." on the
 * dashboard to store the exact spot (saved on the roboRIO).
 */
public class BackupShots {
    public enum Spot {
        MIDSHOT("Midshot", 2.30, 0.0),
        LEFT_TRENCH("Left Trench", 1.05, 3.39),
        RIGHT_TRENCH("Right Trench", 1.05, -3.39);

        final String label;
        final double defaultForward, defaultLeft;

        Spot(String label, double forward, double left) {
            this.label = label;
            this.defaultForward = forward;
            this.defaultLeft = left;
        }

        String key() { return "Backup Spot " + label; }
    }

    private final CommandSwerveDrivetrain drivetrain;
    private final VisionLocalizer localizer;

    public BackupShots(CommandSwerveDrivetrain drivetrain, VisionLocalizer localizer) {
        this.drivetrain = drivetrain;
        this.localizer = localizer;
        for (Spot s : Spot.values()) {
            Preferences.initString(s.key(), String.format(Locale.US, "%.3f,%.3f", s.defaultForward, s.defaultLeft));
        }
        SmartDashboard.putString("Backup Shot", "-");
    }

    private static boolean red() {
        return DriverStation.getAlliance().map(a -> a == Alliance.Red).orElse(false);
    }

    /** {forward, left} of the spot from our hub, in meters. */
    private static double[] relative(Spot s) {
        try {
            String[] p = Preferences.getString(s.key(), "").split(",");
            return new double[] {Double.parseDouble(p[0]), Double.parseDouble(p[1])};
        } catch (Exception e) {
            return new double[] {s.defaultForward, s.defaultLeft};
        }
    }

    /** The spot on the field (blue-origin coordinates) for our alliance. */
    public static Translation2d fieldPosition(Spot s) {
        double[] r = relative(s);
        Translation2d hub = AimSubsystem.ourHub();
        // Red drivers look toward -X (their left is -Y); blue drivers look toward +X (left is +Y).
        return red() ? new Translation2d(hub.getX() + r[0], hub.getY() - r[1])
                     : new Translation2d(hub.getX() - r[0], hub.getY() + r[1]);
    }

    /** Distance (m) from the spot to the hub center (for the shot map plan). */
    public static double distance(Spot s) {
        double[] r = relative(s);
        return Math.hypot(r[0], r[1]);
    }

    /**
     * SHOT PLOTTING without the camera (Oct 27): put the robot's CENTER on a tape mark straight out
     * from the middle of our hub's front face (on the hub's center line), type the mark's distance
     * from the HUB CENTER into "Plot Distance (m)", press "Set Pose: Plot Spot". The robot puts
     * itself exactly there (keeps the gyro heading; unknown heading = facing straight away from our
     * driver station), so "Record Shot Point" records at the true distance. Turn "Vision Pose ON"
     * OFF while plotting so a mis-tilted camera can't move it.
     */
    public Command plotSpotCommand() {
        SmartDashboard.putNumber("Plot Distance (m)", SmartDashboard.getNumber("Plot Distance (m)", 1.5));
        SmartDashboard.putNumber("Plot Angle (deg)", SmartDashboard.getNumber("Plot Angle (deg)", 0.0));
        return Commands.runOnce(() -> {
            double d = SmartDashboard.getNumber("Plot Distance (m)", 1.5);
            if (d < 0.8 || d > 8.0) {
                SmartDashboard.putString("Backup Shot", "PLOT SPOT: distance must be 0.8 to 8 m");
                return;
            }
            // Angle from the straight-out line (hub center toward our driver station): + = toward the
            // driver's LEFT, - = right. 0 = straight out from the middle of the hub's front face.
            double ang = Math.toRadians(SmartDashboard.getNumber("Plot Angle (deg)", 0.0));
            double fwd = d * Math.cos(ang), left = d * Math.sin(ang);
            Translation2d hub = AimSubsystem.ourHub();
            Translation2d spot = red() ? new Translation2d(hub.getX() + fwd, hub.getY() - left)
                                       : new Translation2d(hub.getX() - fwd, hub.getY() + left);
            Rotation2d rot;
            String heading;
            if (localizer.headingKnown()) {
                rot = drivetrain.getState().Pose.getRotation();
                heading = "gyro heading";
            } else {
                rot = red() ? Rotation2d.k180deg : Rotation2d.kZero;
                heading = "heading ASSUMED: facing straight away from our driver station";
            }
            drivetrain.resetPose(new Pose2d(spot, rot));
            localizer.markPoseSetByAuto();
            String msg = String.format(Locale.US, "PLOT SPOT set: %.2f m (%.0f in) from the hub center at %.0f deg (%s)",
                d, d / 0.0254, Math.toDegrees(ang), heading);
            SmartDashboard.putString("Backup Shot", msg);
            System.out.println("PLOT | " + msg);
        }).ignoringDisable(true).withName("Set Pose: Plot Spot");
    }

    /** Operator button: "we are parked on this spot". Seeds the pose and turns auto-aim on. */
    public Command useCommand(Spot s, Command aimCommand) {
        return Commands.runOnce(() -> {
            String heading;
            Rotation2d rot;
            if (localizer.headingKnown()) {
                rot = drivetrain.getState().Pose.getRotation();
                heading = "gyro heading";
            } else {
                // No heading yet: assume the robot is squared up facing away from our driver station.
                rot = red() ? Rotation2d.k180deg : Rotation2d.kZero;
                heading = "heading ASSUMED: robot must face straight away from our driver station";
            }
            drivetrain.resetPose(new Pose2d(fieldPosition(s), rot));
            localizer.markPoseSetByAuto();
            if (!aimCommand.isScheduled()) CommandScheduler.getInstance().schedule(aimCommand);
            String msg = "BACKUP " + s.label.toUpperCase(Locale.US) + " (" + heading + "). Hold RT.";
            SmartDashboard.putString("Backup Shot", msg);
            System.out.println("BACKUP SHOT | " + msg);
            frc.robot.controls.Haptics.on(frc.robot.controls.Haptics.Pad.OPERATOR); // spot set
        }).withName("Backup shot: " + s.label);
    }

    /** Dashboard "Save Spot: ...": park on the spot with the position known, press to store it. */
    public Command saveCommand(Spot s) {
        return Commands.runOnce(() -> {
            if (!localizer.poseTrusted()) {
                SmartDashboard.putString("Backup Shot", "NOT SAVED: position not known yet (wait for OK SET)");
                return;
            }
            Translation2d p = drivetrain.getState().Pose.getTranslation();
            Translation2d hub = AimSubsystem.ourHub();
            double forward = red() ? p.getX() - hub.getX() : hub.getX() - p.getX();
            double left = red() ? hub.getY() - p.getY() : p.getY() - hub.getY();
            Preferences.setString(s.key(), String.format(Locale.US, "%.3f,%.3f", forward, left));
            String msg = String.format(Locale.US, "SAVED %s: %.2f m forward, %.2f m left of the hub (%.2f m away)",
                s.label, forward, left, Math.hypot(forward, left));
            SmartDashboard.putString("Backup Shot", msg);
            System.out.println("BACKUP SHOT | " + msg);
        }).ignoringDisable(true).withName("Save Spot: " + s.label);
    }
}
