package frc.robot.characterization;

import java.util.Locale;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * DRIVE TESTS (dashboard buttons, robot ENABLED in teleop, nobody near the robot). Each one
 * measures a number the drivetrain needs, shows it on "Drive Test" and prints the exact code
 * change as {@code DRIVE TEST | ...} in the Driver Station console. Touching the driver's sticks
 * stops a test at once; clicking its button again stops it too.
 * <ul>
 *   <li><b>Test: Wheel Radius</b> - spins in place slowly about 2 turns (needs ~1 m / 3 ft of
 *       clear space around the robot). Worn tread = smaller wheel = every path ends short.</li>
 *   <li><b>Test: Drive Feedforward</b> - drives straight FORWARD slowly up to ~1 m/s for about
 *       2.5 m (8 ft): needs that much clear carpet in front. Gives kS and kV for the drive motors.</li>
 *   <li><b>Test: Slip Current</b> - put the FRONT bumper flat against a wall first. Pushes harder
 *       and harder until the wheels spin. Above that current the wheels only spin.</li>
 * </ul>
 */
public final class DriveTests {
    private static final String kOut = "Drive Test";
    private final CommandSwerveDrivetrain drive;
    private final DoubleSupplier driverStick;
    private final double codeRadius = TunerConstants.FrontLeft.WheelRadius;       // m
    private final double gearRatio = TunerConstants.FrontLeft.DriveMotorGearRatio;

    private final SwerveRequest.RobotCentric spin = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.Velocity);
    private final SwerveRequest.SysIdSwerveTranslation volts = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

    /** @param driverStick the driver's stick deflection (0-1): any input stops a test */
    public DriveTests(CommandSwerveDrivetrain drive, DoubleSupplier driverStick) {
        this.drive = drive;
        this.driverStick = driverStick;
        SmartDashboard.putString(kOut, "-");
    }

    private boolean driverTouched() { return driverStick.getAsDouble() > 0.15; }

    private double avgWheelSpeed() {
        double s = 0;
        var st = drive.getState().ModuleStates;
        for (var m : st) s += Math.abs(m.speedMetersPerSecond);
        return s / st.length;
    }

    private double avgDriveAmps() {
        double a = 0;
        var mods = drive.getModules();
        for (var m : mods) a += Math.abs(m.getDriveMotor().getStatorCurrent().getValueAsDouble());
        return a / mods.length;
    }

    private double[] wheelDistances() {
        var pos = drive.getState().ModulePositions;
        double[] d = new double[pos.length];
        for (int i = 0; i < pos.length; i++) d[i] = pos[i].distanceMeters;
        return d;
    }

    private static void say(String s) {
        SmartDashboard.putString(kOut, s);
        System.out.println("DRIVE TEST | " + s);
    }

    // ---------------------------------------------------------------- wheel radius

    public Command wheelRadiusCommand() {
        final double maxRate = 0.8;   // rad/s (wheels ~0.3 m/s: slow enough not to slip)
        final double settle = 1.5;    // s of spinning before measuring starts
        final double turns = 2.0;
        SlewRateLimiter ramp = new SlewRateLimiter(0.4);
        double[] startDist = new double[4];
        double[] startYaw = {0};
        double[] t0 = {0};
        boolean[] measuring = {false};
        double[] result = {Double.NaN};
        double baseRadius = drive.getModuleLocations()[0].getNorm();

        return Commands.runEnd(() -> {
                double t = Timer.getFPGATimestamp() - t0[0];
                drive.setControl(spin.withVelocityX(0).withVelocityY(0).withRotationalRate(ramp.calculate(maxRate)));
                double yaw = Math.toRadians(drive.getPigeon2().getYaw().getValueAsDouble());
                if (!measuring[0] && t >= settle) {
                    measuring[0] = true;
                    startYaw[0] = yaw;
                    System.arraycopy(wheelDistances(), 0, startDist, 0, 4);
                }
                if (measuring[0]) {
                    double turned = yaw - startYaw[0];
                    double[] now = wheelDistances();
                    double[] dd = new double[4];
                    for (int i = 0; i < 4; i++) dd[i] = now[i] - startDist[i];
                    result[0] = WheelRadiusCalc.radius(codeRadius, baseRadius, turned, dd);
                    SmartDashboard.putString(kOut, String.format(Locale.US, "Wheel radius: spinning, %.0f of %.0f deg",
                        Math.toDegrees(Math.abs(turned)), turns * 360));
                } else {
                    SmartDashboard.putString(kOut, "Wheel radius: speeding up...");
                }
            }, () -> {
                drive.setControl(idle);
                if (Double.isNaN(result[0])) {
                    say("Wheel radius: stopped too early, no result (needs a full turn or more)");
                } else {
                    double in = result[0] / 0.0254;
                    say(String.format(Locale.US,
                        "Wheel radius %.3f in (code uses %.3f in, %+.1f%%). Set TunerConstants kWheelRadius = Inches.of(%.3f) "
                        + "and PathPlanner Settings > Wheel Radius = %.4f m. Run it 2-3 times; use the average.",
                        in, codeRadius / 0.0254, 100 * (result[0] / codeRadius - 1), in, result[0]));
                }
            }, drive)
            .beforeStarting(() -> {
                t0[0] = Timer.getFPGATimestamp();
                measuring[0] = false;
                result[0] = Double.NaN;
                ramp.reset(0);
            })
            .until(() -> driverTouched() || (measuring[0]
                && Math.abs(Math.toRadians(drive.getPigeon2().getYaw().getValueAsDouble()) - startYaw[0]) >= turns * 2 * Math.PI))
            .withTimeout(30)
            .withName("Test: Wheel Radius");
    }

    // ---------------------------------------------------------------- feedforward

    public Command feedforwardCommand() {
        final double rampVoltsPerSec = 0.5;
        final double maxVolts = 3.0;      // ~1 m/s
        final double maxDistance = 2.5;   // m (8 ft)
        FeedforwardFit[] fit = {null};
        double[] t0 = {0};
        Translation2d[] start = {null};

        return Commands.runEnd(() -> {
                double v = Math.min(maxVolts, rampVoltsPerSec * (Timer.getFPGATimestamp() - t0[0]));
                drive.setControl(volts.withVolts(v));
                fit[0].add(v, avgWheelSpeed());
                SmartDashboard.putString(kOut, String.format(Locale.US, "Feedforward: %.2f V, %.2f m/s", v, avgWheelSpeed()));
            }, () -> {
                drive.setControl(idle);
                double kv = fit[0].kV(), ks = fit[0].kS();
                if (Double.isNaN(kv)) {
                    say("Feedforward: not enough data (" + fit[0].samples() + " moving samples): let it run longer");
                } else {
                    say(String.format(Locale.US,
                        "Drive kS %.3f V, kV %.3f V per m/s = %.4f V per motor rot/s. Set TunerConstants driveGains "
                        + ".withKS(%.3f).withKV(%.4f) (now kS 0, kV 0.124).",
                        ks, kv, FeedforwardFit.perMotorRps(kv, codeRadius, gearRatio), ks,
                        FeedforwardFit.perMotorRps(kv, codeRadius, gearRatio)));
                }
            }, drive)
            .beforeStarting(() -> {
                t0[0] = Timer.getFPGATimestamp();
                fit[0] = new FeedforwardFit(0.05);
                start[0] = drive.getState().Pose.getTranslation();
            })
            .until(() -> driverTouched()
                || drive.getState().Pose.getTranslation().getDistance(start[0]) > maxDistance
                || Timer.getFPGATimestamp() - t0[0] > maxVolts / rampVoltsPerSec + 1.0)
            .withName("Test: Drive Feedforward");
    }

    // ---------------------------------------------------------------- slip current

    /**
     * Drive motor stator limit (A) for every module. The robot normally runs at its slip current;
     * the slip test raises it for the test so it can find where the wheels really break loose.
     */
    private void setDriveStatorLimit(double amps) {
        for (var m : drive.getModules()) {
            var c = new com.ctre.phoenix6.configs.CurrentLimitsConfigs();
            var cfg = m.getDriveMotor().getConfigurator();
            cfg.refresh(c);
            c.StatorCurrentLimit = amps;
            c.StatorCurrentLimitEnable = true;
            cfg.apply(c);
        }
    }

    public Command slipCurrentCommand() {
        final double rampVoltsPerSec = 0.5;
        final double maxVolts = 8.0;
        final double testLimit = 120.0;   // A, only during this test
        final double normalLimit = TunerConstants.FrontLeft.SlipCurrent;
        SlipDetector[] det = {null};
        double[] t0 = {0};

        return Commands.runEnd(() -> {
                double v = Math.min(maxVolts, rampVoltsPerSec * (Timer.getFPGATimestamp() - t0[0]));
                drive.setControl(volts.withVolts(v));
                det[0].update(avgDriveAmps(), avgWheelSpeed());
                SmartDashboard.putString(kOut, String.format(Locale.US, "Slip: %.1f V, %.0f A, wheels %.2f m/s",
                    v, avgDriveAmps(), avgWheelSpeed()));
            }, () -> {
                drive.setControl(idle);
                setDriveStatorLimit(normalLimit);
                double a = det[0].slipAmps();
                if (Double.isNaN(a)) {
                    say(String.format(Locale.US, "Slip: wheels never broke loose (peak %.0f A). Ran out of voltage or stopped early.",
                        det[0].peakAmps()));
                } else if (a < 30) {
                    say(String.format(Locale.US, "Slip: the robot MOVED at %.0f A: put the front bumper flat against a wall and run again", a));
                } else {
                    say(String.format(Locale.US,
                        "Slip current %.0f A. Set TunerConstants kSlipCurrent = Amps.of(%.0f) and PathPlanner Settings "
                        + "> Drive Current Limit = %.0f. Run it 2-3 times; use the LOWEST.", a, a, a));
                }
            }, drive)
            .beforeStarting(() -> {
                t0[0] = Timer.getFPGATimestamp();
                det[0] = new SlipDetector(0.3);
                setDriveStatorLimit(testLimit);
            })
            .until(() -> driverTouched() || det[0].update(avgDriveAmps(), avgWheelSpeed())
                || Timer.getFPGATimestamp() - t0[0] > maxVolts / rampVoltsPerSec + 1.0)
            .withName("Test: Slip Current");
    }
}
