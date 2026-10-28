package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Defense;
import frc.robot.defense.PinClock;
import frc.robot.defense.TowerGuard;

/**
 * DEFENSE MODE (Driver Y toggles it). Turning it on:
 * <ul>
 *   <li>stops auto-aim and vacuum mode, raises the intake and lowers the hood: nothing sticks out
 *       to get broken or to reach inside an opponent's frame (G415 MAJOR + YELLOW CARD when our
 *       bumpers are outside our alliance zone);</li>
 *   <li>the drive answers the sticks about twice as fast (quicker to mirror an opponent);</li>
 *   <li>sticks centered = wheels lock in an X, so the robot is hard to push out of a lane.</li>
 * </ul>
 * While it's on it also warns the driver (rumble + dashboard "Defense"):
 * <ul>
 *   <li>G418 pins: pushing into something without moving for 2 s = strong pulses: back off before 3 s;</li>
 *   <li>G420 (always, defense mode or not): last 30 s near THEIR tower = steady rumble: don't touch
 *       anyone on it (MAJOR FOUL, and they get LEVEL 3 points).</li>
 * </ul>
 * It turns itself off when the operator turns auto-aim or vacuum back on.
 */
public class DefenseMode extends SubsystemBase {
    private static volatile boolean on = false;
    private static volatile double driverStick = 0;

    private final CommandSwerveDrivetrain drivetrain;
    private final AimSubsystem aim;
    private final PinClock pin = new PinClock(Defense.kPinStick, Defense.kPinMaxSpeed, Defense.kPinAmps,
        Defense.kPinStartSeconds, Defense.kPinResetSeconds);
    private double pinSeconds = 0;
    private boolean towerWarning = false;

    public DefenseMode(CommandSwerveDrivetrain drivetrain, AimSubsystem aim) {
        this.drivetrain = drivetrain;
        this.aim = aim;
        SmartDashboard.putBoolean("Defense Mode", false);
        SmartDashboard.putString("Defense", "OFF (Driver Y)");
    }

    /** Defense mode is on (the drive command reads this). */
    public static boolean active() { return on; }

    /** The driver's translate stick (0-1), as last reported by the drive command. */
    public static double driverStick() { return driverStick; }

    /** The drive command reports the driver's translate stick (0-1) every loop. */
    public static void reportDriverStick(double magnitude) { driverStick = magnitude; }

    /**
     * Driver Y: toggle. Turning on runs `onEnable` (stop aim/vacuum, intake up, hood down).
     */
    public Command toggleCommand(Runnable onEnable) { return toggleCommand(onEnable, () -> {}); }

    /** Driver Y: toggle; `onDisable` runs when Y turns it OFF. */
    public Command toggleCommand(Runnable onEnable, Runnable onDisable) {
        return Commands.runOnce(() -> {
            on = !on;
            changedAt = Timer.getFPGATimestamp();
            changedOn = on;
            if (on) onEnable.run(); else onDisable.run();
            System.out.println("DEFENSE | " + (on ? "ON" : "OFF"));
        }).withName("Toggle defense mode");
    }

    /** Pin warning: rumble the driver now (strong pulses). */
    public boolean pinWarning() { return on && pinSeconds >= Defense.kPinWarnSeconds; }

    /** Last 30 s near their tower: rumble the driver (steady). */
    public boolean towerWarning() { return towerWarning; }

    private static double changedAt = Double.NEGATIVE_INFINITY;
    private static boolean changedOn = false;

    /**
     * The driver FEELS every defense change, so the Y toggle never gets out of step (audit Oct 27):
     * ON = one long buzz; OFF (Driver Y, or by itself when the operator turns aim/vacuum on) = two
     * short buzzes. Without it, defense ending by itself and then a Y press turned it back ON.
     */
    public boolean changeBuzz() {
        double t = Timer.getFPGATimestamp() - changedAt;
        if (changedOn) return t < 0.4;
        return t < 0.6 && (t < 0.2 || t > 0.4);
    }

    /** A match is starting: defense mode off (it used to survive a disable). */
    public static void reset() {
        if (on) System.out.println("DEFENSE | OFF (match start)");
        on = false;
    }

    private static boolean red() {
        Optional<Alliance> a = DriverStation.getAlliance();
        return a.isPresent() && a.get() == Alliance.Red;
    }

    @Override
    public void periodic() {
        // The operator wants to shoot again: defense is over.
        if (on && aim.isActive()) {
            on = false;
            changedAt = Timer.getFPGATimestamp();
            changedOn = false;
            System.out.println("DEFENSE | OFF (auto-aim or vacuum turned on)");
        }
        var state = drivetrain.getState();
        ChassisSpeeds s = state.Speeds;
        double speed = Math.hypot(s.vxMetersPerSecond, s.vyMetersPerSecond);
        double amps = 0;
        var modules = drivetrain.getModules();
        for (var m : modules) amps += Math.abs(m.getDriveMotor().getStatorCurrent().getValueAsDouble());
        amps /= Math.max(1, modules.length);

        double now = Timer.getFPGATimestamp();
        pinSeconds = on && DriverStation.isTeleopEnabled() ? pin.update(now, driverStick, speed, amps)
            : pin.update(now, 0, speed, 0);
        var p = state.Pose.getTranslation();
        towerWarning = DriverStation.isTeleopEnabled() && aim.poseUsable()
            && TowerGuard.endgame(DriverStation.getMatchTime())
            && TowerGuard.nearTheirTower(p.getX(), p.getY(), red(), Defense.kTowerMargin);

        String text;
        if (towerWarning) text = "G420: last 30 s at THEIR tower: don't touch robots on it";
        else if (!on) text = "OFF (Driver Y)";
        else if (pinWarning()) text = String.format(java.util.Locale.US, "PIN %.1f s: BACK OFF (G418: 3 s max)", pinSeconds);
        else if (pinSeconds > 0) text = String.format(java.util.Locale.US, "pushing %.1f s", pinSeconds);
        else text = "ON: intake up, X-lock when sticks centered";
        SmartDashboard.putBoolean("Defense Mode", on);
        SmartDashboard.putString("Defense", text);
        SmartDashboard.putNumber("Defense Pin Seconds", pinSeconds);
        SmartDashboard.putNumber("Drive Avg Amps", amps);
        dev.doglog.DogLog.log("Defense/On", on);
        dev.doglog.DogLog.log("Defense/PinSeconds", pinSeconds);
        dev.doglog.DogLog.log("Defense/TowerWarning", towerWarning);
    }
}
