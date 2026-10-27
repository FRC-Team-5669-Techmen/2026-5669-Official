package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Agitation;
import frc.robot.feeding.AgitationPlanner;
import frc.robot.feeding.ShotDetector;

/**
 * AUTOMATIC AGITATION while shooting (teleop and autos, no buttons). Runs every loop:
 * <ul>
 *   <li>Counts balls leaving from the flywheel's dips (ShotDetector).</li>
 *   <li>Feeding but no ball for a moment = the pile has bridged: PUMP the deployed intake
 *       (lift part way, drop, twice) while the roller blinks out then pulls in. Intake stowed or
 *       busy: SHAKE the spindexer (short reverse) instead.</li>
 *   <li>Spindexer stalls (inlet jam): IndexSubsystem.feed() reverses it for a moment by itself.
 *       Jams that keep coming also trigger a pump.</li>
 *   <li>Several agitations and still no ball: the hopper is probably empty, it stops.</li>
 * </ul>
 * The driver always wins: any A press or intake jog interrupts a pump, and the roller pulse is
 * skipped while the driver holds a roller trigger.
 *
 * <p>Dashboard: "Auto Agitate" (on/off, saved), "Agitate Lift (rot)" (saved), "Agitator" (what it
 * is doing), "Last Volley" (balls, seconds, rate, longest gap, pumps, shakes, unjams). Has no
 * motors of its own, so nothing ever "requires" it.
 */
public class Agitator extends SubsystemBase {
    private static final String kEnabledKey = "Auto Agitate";
    private static final String kLiftKey = "Agitate Lift (rot)";

    private final ShooterSubsystem shooter;
    private final IndexSubsystem index;
    private final GroundIntakePivotSubsystem pivot;
    private final IntakeRollerSubsystem roller;

    private final ShotDetector shots = new ShotDetector(
        Agitation.kShotDropRpm, Agitation.kShotRearmRpm, Agitation.kShotMaxDipSeconds, Agitation.kMinShooterRpm);
    private final AgitationPlanner planner = new AgitationPlanner(
        Agitation.kStarveSeconds, Agitation.kPumpCooldown, Agitation.kShakeSeconds,
        Agitation.kMaxPumpsWithoutShot, Agitation.kJamBurstWindow, Agitation.kJamBurstCount);

    private Command pump = null;
    private int pumps = 0, shakes = 0;

    public Agitator(ShooterSubsystem shooter, IndexSubsystem index,
                    GroundIntakePivotSubsystem pivot, IntakeRollerSubsystem roller) {
        this.shooter = shooter;
        this.index = index;
        this.pivot = pivot;
        this.roller = roller;
        Preferences.initBoolean(kEnabledKey, true);
        Preferences.initDouble(kLiftKey, Agitation.kPumpLift);
        SmartDashboard.putBoolean(kEnabledKey, Preferences.getBoolean(kEnabledKey, true));
        SmartDashboard.putNumber(kLiftKey, Preferences.getDouble(kLiftKey, Agitation.kPumpLift));
        SmartDashboard.putString("Last Volley", "-");
    }

    private boolean switchOn() {
        boolean on = SmartDashboard.getBoolean(kEnabledKey, true);
        if (on != Preferences.getBoolean(kEnabledKey, true)) Preferences.setBoolean(kEnabledKey, on);
        return on;
    }

    private double lift() {
        double v = edu.wpi.first.math.MathUtil.clamp(SmartDashboard.getNumber(kLiftKey, Agitation.kPumpLift), 0.3, 4.0);
        if (v != Preferences.getDouble(kLiftKey, Agitation.kPumpLift)) Preferences.setDouble(kLiftKey, v);
        return v;
    }

    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();
        double rpm = shooter.getCurrentRpm();
        shots.update(now, rpm);

        boolean pumping = pump != null && pump.isScheduled();
        boolean pivotFree = CommandScheduler.getInstance().requiring(pivot) == null;
        AgitationPlanner.Action action = planner.update(now,
            switchOn() && DriverStation.isEnabled(),
            index.isFeeding(),
            rpm >= Agitation.kMinShooterRpm,
            shots.shots(), index.jamCount(),
            pivot.isDown(), pivotFree, pumping);

        if (action == AgitationPlanner.Action.PUMP) {
            pump = pivot.pumpCommand(lift(), Agitation.kPumpCycles, Agitation.kPumpLegTimeout);
            CommandScheduler.getInstance().schedule(pump);
            pumps++;
            // Roller: blink out (frees the pile at the bumper), then pull in until the pump ends.
            // Skipped while the driver is using the roller.
            if (CommandScheduler.getInstance().requiring(roller) == null) {
                Command p = pump;
                CommandScheduler.getInstance().schedule(Commands.sequence(
                        Commands.runOnce(() -> roller.setVolts(Agitation.kRollerOutVolts), roller),
                        Commands.waitSeconds(Agitation.kRollerOutSeconds),
                        Commands.run(() -> roller.setVolts(Agitation.kRollerInVolts), roller)
                            .until(() -> !p.isScheduled()))
                    .withTimeout(3.0)
                    .finallyDo(roller::stop)
                    .withName("Agitate: roller pulse"));
            }
            dev.doglog.DogLog.log("Feed/Agitation", "PUMP");
        } else if (action == AgitationPlanner.Action.SHAKE) {
            index.shake(Agitation.kShakeSeconds);
            shakes++;
            dev.doglog.DogLog.log("Feed/Agitation", "SHAKE");
        }

        SmartDashboard.putString("Agitator", planner.status());
        SmartDashboard.putString("Last Volley", planner.lastVolley());
        SmartDashboard.putNumber("Agitator Balls Counted", shots.shots());
        SmartDashboard.putNumber("Agitator Pumps", pumps);
        SmartDashboard.putNumber("Agitator Shakes", shakes);
        SmartDashboard.putNumber("Spindex Unjams", index.jamCount());
        dev.doglog.DogLog.log("Feed/Balls", shots.shots());
        dev.doglog.DogLog.log("Feed/Unjams", index.jamCount());
    }
}
