package frc.robot.subsystems;

import java.util.ArrayDeque;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Agitation;
import frc.robot.Constants.IntakeRoller;
import frc.robot.feeding.AgitationPlanner;
import frc.robot.feeding.ShotDetector;

/**
 * AUTOMATIC AGITATION while shooting (teleop and autos, no buttons). Every loop:
 * <ul>
 *   <li>Counts balls leaving from the flywheel's dips (ShotDetector).</li>
 *   <li>PREVENT: while balls are fed and the intake is down, the roller runs in at FULL power
 *       (IntakeRoller.kRollerVolts, Oct 31; it used to be a gentle 5 V) to keep the pile moving.</li>
 *   <li>Escalation ladder on a gap (AgitationPlanner): NUDGE (a spindexer shake; the roller is
 *       never tapped out anymore), then PUMP the intake, then (switch on) SHIMMY the drivetrain,
 *       then stop if the hopper looks empty.</li>
 *   <li>Spindexer stalls are reversed by IndexSubsystem.feed() itself.</li>
 *   <li>PROTECT: no pumps when a pivot motor is hot (over 80 C until under 65 C), over 12 pumps
 *       a minute, or with the battery under 8.5 V; no shimmy on a low battery.</li>
 * </ul>
 * The driver always wins: A or the intake jog interrupts a pump, the roller is left alone while
 * the driver holds a roller trigger, and any stick input cancels a shimmy.
 */
public class Agitator extends SubsystemBase {
    private static final String kEnabledKey = "Auto Agitate";
    private static final String kLiftKey = "Agitate Lift (rot)";
    private static final String kShimmyKey = "Agitate Drive Shimmy";

    private final ShooterSubsystem shooter;
    private final IndexSubsystem index;
    private final GroundIntakePivotSubsystem pivot;
    private final IntakeRollerSubsystem roller;

    private final ShotDetector shots = new ShotDetector(
        Agitation.kShotDropRpm, Agitation.kShotRearmRpm, Agitation.kShotMaxDipSeconds, Agitation.kMinShooterRpm);
    private final AgitationPlanner planner;
    private final AgitationPlanner.Inputs in = new AgitationPlanner.Inputs();

    private Command pump = null;
    private boolean wasEmpty = false;
    private final ArrayDeque<Double> pumpTimes = new ArrayDeque<>();
    private boolean hot = false;
    private int nudges = 0, pumps = 0, shakes = 0, shimmies = 0;

    // Drivetrain shimmy: read by the drive's default command and by AimSubsystem (lead off).
    private static volatile double shimmyStart = Double.NEGATIVE_INFINITY;
    private static volatile double shimmyUntil = Double.NEGATIVE_INFINITY;
    private static volatile boolean driverHandsOff = true;

    public Agitator(ShooterSubsystem shooter, IndexSubsystem index,
                    GroundIntakePivotSubsystem pivot, IntakeRollerSubsystem roller) {
        this.shooter = shooter;
        this.index = index;
        this.pivot = pivot;
        this.roller = roller;
        AgitationPlanner.Params p = new AgitationPlanner.Params();
        p.nudgeSeconds = Agitation.kNudgeSeconds;
        p.pumpSeconds = Agitation.kStarveSeconds;
        p.cooldown = Agitation.kPumpCooldown;
        p.shakeSeconds = Agitation.kShakeSeconds;
        p.maxTries = Agitation.kMaxPumpsWithoutShot;
        p.burstWindow = Agitation.kJamBurstWindow;
        p.burstCount = Agitation.kJamBurstCount;
        planner = new AgitationPlanner(p);

        Preferences.initBoolean(kEnabledKey, true);
        Preferences.initDouble(kLiftKey, Agitation.kPumpLift);
        Preferences.initBoolean(kShimmyKey, false); // untested on the robot: off until the team tries it
        SmartDashboard.putBoolean(kEnabledKey, Preferences.getBoolean(kEnabledKey, true));
        SmartDashboard.putNumber(kLiftKey, Preferences.getDouble(kLiftKey, Agitation.kPumpLift));
        SmartDashboard.putBoolean(kShimmyKey, Preferences.getBoolean(kShimmyKey, false));
        SmartDashboard.putString("Last Volley", "-");
    }

    // ---------- shimmy hooks (drive default command, AimSubsystem) ----------

    /** True while the drivetrain shimmy owns the wheels. */
    public static boolean shimmying() {
        return Timer.getFPGATimestamp() < shimmyUntil && driverHandsOff;
    }

    /** Robot-relative forward speed for the shimmy right now (m/s), 0 when not shimmying. */
    public static double shimmySpeed() {
        if (!shimmying()) return 0.0;
        double t = Timer.getFPGATimestamp() - shimmyStart;
        return Agitation.kShimmySpeed * Math.sin(Agitation.kShimmyRate * t);
    }

    /** The drive command reports the driver's sticks every loop; any input cancels a shimmy. */
    public static void reportDriverSticks(double x, double y, double r) {
        double db = Agitation.kShimmyStickDeadband;
        driverHandsOff = Math.abs(x) < db && Math.abs(y) < db && Math.abs(r) < db;
        if (!driverHandsOff) shimmyUntil = Double.NEGATIVE_INFINITY;
    }

    // ---------- dashboard settings (saved on the roboRIO) ----------

    private static boolean savedSwitch(String key, boolean dflt) {
        boolean v = SmartDashboard.getBoolean(key, dflt);
        if (v != Preferences.getBoolean(key, dflt)) Preferences.setBoolean(key, v);
        return v;
    }

    private static double savedNumber(String key, double dflt, double lo, double hi) {
        double v = MathUtil.clamp(SmartDashboard.getNumber(key, dflt), lo, hi);
        if (v != Preferences.getDouble(key, dflt)) Preferences.setDouble(key, v);
        return v;
    }

    private boolean rollerFree() {
        return CommandScheduler.getInstance().requiring(roller) == null;
    }

    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();
        double rpm = shooter.getCurrentRpm();
        shots.update(now, rpm);

        boolean on = savedSwitch(kEnabledKey, true);
        double lift = savedNumber(kLiftKey, Agitation.kPumpLift, 0.3, 4.0);
        boolean shimmyOn = savedSwitch(kShimmyKey, false);
        boolean enabled = on && DriverStation.isEnabled();

        // ---- protection ----
        double temp = Math.max(pivot.tempCelsius(), index.tempCelsius());
        if (temp > Agitation.kHotCelsius) hot = true;
        if (temp < Agitation.kCoolCelsius) hot = false;
        while (!pumpTimes.isEmpty() && now - pumpTimes.peekFirst() > 60.0) pumpTimes.removeFirst();
        boolean batteryOk = RobotController.getBatteryVoltage() > Agitation.kMinBatteryVolts;

        boolean pumping = pump != null && pump.isScheduled();
        boolean feeding = index.isFeeding();
        in.enabled = enabled;
        in.feeding = feeding;
        in.shooterUp = rpm >= Agitation.kMinShooterRpm;
        in.shots = shots.shots();
        in.jams = index.jamCount();
        boolean climbing = ElevatorSubsystem.isRaised();
        if (climbing) {                       // stop anything already moving the intake or robot
            if (pump != null && pump.isScheduled()) pump.cancel();
            shimmyUntil = Double.NEGATIVE_INFINITY;
        }
        // Dump mode: never pump the intake (it's X-taking, maybe under the trench) or shimmy the drive.
        boolean dumping = frc.robot.commands.VacuumCommand.dumping();
        // Vacuum: never lift the intake while the driver is collecting (Oct 28 audit).
        boolean vacuuming = frc.robot.commands.VacuumCommand.vacuuming();
        in.pumpPossible = !climbing && !dumping && !vacuuming && pivot.isDown() && CommandScheduler.getInstance().requiring(pivot) == null
            && !hot && batteryOk && pumpTimes.size() < Agitation.kMaxPumpsPerMinute;
        in.shimmyPossible = !climbing && !dumping && shimmyOn && DriverStation.isTeleopEnabled() && driverHandsOff && batteryOk && !pumping;
        in.busy = pumping || shimmying();
        AgitationPlanner.Action action = planner.update(now, in);
        // Hopper looks EMPTY: three long buzzes for the operator, once per volley.
        boolean empty = planner.probablyEmpty();
        if (empty && !wasEmpty) frc.robot.controls.Haptics.empty(frc.robot.controls.Haptics.Pad.OPERATOR);
        wasEmpty = empty;

        switch (action) {
            case NUDGE -> {
                nudges++;
                index.shake(Agitation.kShakeSeconds);
            }
            case PUMP -> {
                pumps++;
                pumpTimes.addLast(now);
                pump = pivot.pumpCommand(lift, Agitation.kPumpCycles, Agitation.kPumpLegTimeout);
                CommandScheduler.getInstance().schedule(pump);
            }
            case SHAKE -> { shakes++; index.shake(Agitation.kShakeSeconds); }
            case SHIMMY -> {
                shimmies++;
                shimmyStart = now;
                shimmyUntil = now + Agitation.kShimmySeconds;
            }
            default -> { }
        }
        if (action != AgitationPlanner.Action.NONE) dev.doglog.DogLog.log("Feed/Agitation", action.name());

        // ---- prevention: full-power roller-in while balls are fed (intake down or pumping, roller free) ----
        if (rollerFree()) {
            if (enabled && feeding && (pivot.isDown() || pumping)) {
                roller.setVolts(IntakeRoller.kRollerVolts);
                assisting = true;
            } else if (assisting) {
                roller.stop();
                assisting = false;
            }
        }

        SmartDashboard.putString("Agitator", hot ? "HOT pivot/spindexer motor: no pumps until it cools"
            : !batteryOk && feeding ? "battery low: no pumps" : planner.status());
        SmartDashboard.putString("Last Volley", planner.lastVolley());
        SmartDashboard.putNumber("Agitator Balls Counted", shots.shots());
        SmartDashboard.putNumber("Agitator Nudges", nudges);
        SmartDashboard.putNumber("Agitator Pumps", pumps);
        SmartDashboard.putNumber("Agitator Shakes", shakes);
        SmartDashboard.putNumber("Agitator Shimmies", shimmies);
        SmartDashboard.putNumber("Spindex Unjams", index.jamCount());
        SmartDashboard.putNumber("Agitator Max Motor Temp (C)", temp);
        dev.doglog.DogLog.log("Feed/Balls", shots.shots());
        dev.doglog.DogLog.log("Feed/Unjams", index.jamCount());
        dev.doglog.DogLog.log("Feed/MaxTempC", temp);
    }

    private boolean assisting = false;
}
