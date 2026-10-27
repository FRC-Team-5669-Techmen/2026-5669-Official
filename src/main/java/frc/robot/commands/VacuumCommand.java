package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.AimSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GoobaSubsystem;
import frc.robot.subsystems.Goober;
import frc.robot.subsystems.GroundIntakePivotSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.ShooterIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * VACUUM MODE (Operator Back toggles it): intake and shoot at the same time, hands-free.
 * <ul>
 *   <li>Intake goes DOWN (unless climbing) and the roller pulls balls in.</li>
 *   <li>Auto-aim turns on; the shooter spins to the table speed for the current target.</li>
 *   <li>It FIRES BY ITSELF whenever ready (no trigger), so the driver just drives through balls.</li>
 *   <li>Target follows "Shot Target Mode" (Operator Start): AUTO scores from inside our alliance
 *       zone and PASSES home from the neutral zone; SCORE-only holds balls outside the zone;
 *       PASS-only always passes.</li>
 * </ul>
 * The drive team always wins: Driver RT/LT take the roller, Operator RT/B take the feeders, the
 * hood/turret jogs take aim, each only while held; vacuum picks every piece back up afterwards.
 * Operator RB (aim off) or Y (manual hood) turn vacuum mode OFF. It never fires without a known
 * position (it can't check the G407 zone line without one), and stops the roller while climbing.
 */
public class VacuumCommand extends Command {
    private final AimSubsystem aim;
    private final GroundIntakePivotSubsystem pivot;
    private final IntakeRollerSubsystem roller;
    private final Command aimCommand;
    private final Command shoot;
    private final Command rollerIn;
    private boolean startedAim = false;

    public VacuumCommand(AimSubsystem aim, ShooterSubsystem shooter, IndexSubsystem index,
                         ShooterIntakeSubsystem uptake, Goober turret, GoobaSubsystem hood,
                         GroundIntakePivotSubsystem pivot, IntakeRollerSubsystem roller, Command aimCommand) {
        this.aim = aim;
        this.pivot = pivot;
        this.roller = roller;
        this.aimCommand = aimCommand;
        this.shoot = new ShootCommand(aim, shooter, index, uptake, turret, hood, true, false).withName("Vacuum: shoot");
        this.rollerIn = roller.runCommand().withName("Vacuum: roller in");
        // No requirements of its own: it runs the pieces as separate commands, so the driver's
        // roller triggers interrupt only the roller, never the whole mode.
    }

    @Override
    public void initialize() {
        aim.setVacuum(true);
        if (!ElevatorSubsystem.isRaised()) pivot.deployIfUp();
        startedAim = !aimCommand.isScheduled();
        if (startedAim) CommandScheduler.getInstance().schedule(aimCommand);
        CommandScheduler.getInstance().schedule(shoot);
        SmartDashboard.putBoolean("Vacuum Mode", true);
        System.out.println("VACUUM | on: intake + auto-shoot (" + aim.currentMode() + ")");
    }

    @Override
    public void execute() {
        // Never fight the drive team: each piece only comes back once nobody else is using its
        // mechanisms (Driver RT/LT on the roller; Operator RT/B on the feeders; hood/turret jogs).
        boolean climbing = ElevatorSubsystem.isRaised();
        if (climbing && rollerIn.isScheduled()) rollerIn.cancel();      // climbing: no roller
        if (!climbing && !rollerIn.isScheduled() && free(rollerIn)) CommandScheduler.getInstance().schedule(rollerIn);
        if (!aimCommand.isScheduled() && free(aimCommand)) CommandScheduler.getInstance().schedule(aimCommand);
        if (!shoot.isScheduled() && free(shoot)) CommandScheduler.getInstance().schedule(shoot);
    }

    /** Every mechanism this command needs is idle (or only running its default command). */
    private static boolean free(Command c) {
        CommandScheduler cs = CommandScheduler.getInstance();
        for (var sub : c.getRequirements()) {
            Command using = cs.requiring(sub);
            if (using != null && using != cs.getDefaultCommand(sub)) return false;
        }
        return true;
    }

    @Override
    public boolean isFinished() { return false; }

    @Override
    public void end(boolean interrupted) {
        aim.setVacuum(false);
        shoot.cancel();
        rollerIn.cancel();
        if (startedAim) aimCommand.cancel();
        SmartDashboard.putBoolean("Vacuum Mode", false);
        System.out.println("VACUUM | off");
    }
}
