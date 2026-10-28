package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AimSubsystem;
import frc.robot.subsystems.GoobaSubsystem;
import frc.robot.subsystems.Goober;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShooterIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * SHOOT FROM ANYWHERE firing (Operator RT while auto-aim is on, and autos). Spins the shooter to
 * the shot table's speed for the distance and feeds balls (Spindex + uptake) ONLY while everything
 * is ready: turret locked on, shooter at speed, hood in place. When it stops being ready (robot
 * moved, turret catching up) feeding pauses by itself, so no ball is wasted.
 */
public class ShootCommand extends Command {
    private final AimSubsystem aim;
    private final ShooterSubsystem shooter;
    private final IndexSubsystem index;
    private final ShooterIntakeSubsystem uptake;
    private final Goober turret;
    private final GoobaSubsystem hood;
    private final boolean spinShooter;
    private final boolean stopShooterOnEnd;
    private boolean firing = false;

    /**
     * @param spinShooter true in autos: this command spins the shooter. false in teleop: the
     *     shooter's default command already spins it to the table speed while LT or RT is held,
     *     so releasing RT doesn't stop it while LT is still held.
     */
    public ShootCommand(AimSubsystem aim, ShooterSubsystem shooter, IndexSubsystem index,
                        ShooterIntakeSubsystem uptake, Goober turret, GoobaSubsystem hood, boolean spinShooter) {
        this(aim, shooter, index, uptake, turret, hood, spinShooter, true);
    }

    /** @param stopShooterOnEnd false for vacuum mode: an unjam (B) interrupting it must not spin the wheel down */
    public ShootCommand(AimSubsystem aim, ShooterSubsystem shooter, IndexSubsystem index,
                        ShooterIntakeSubsystem uptake, Goober turret, GoobaSubsystem hood, boolean spinShooter,
                        boolean stopShooterOnEnd) {
        this.stopShooterOnEnd = stopShooterOnEnd;
        this.aim = aim;
        this.shooter = shooter;
        this.index = index;
        this.uptake = uptake;
        this.turret = turret;
        this.hood = hood;
        this.spinShooter = spinShooter;
        // NOT the turret/hood: AimCommand moves those.
        if (spinShooter) {
            addRequirements(shooter, index, uptake);
        } else {
            addRequirements(index, uptake);
        }
    }

    @Override
    public void initialize() {
        firing = false;
    }

    @Override
    public void execute() {
        if (spinShooter) {
            shooter.runAtRPM(aim.targetRpm());
        }
        // Start when everything is ready; after that keep feeding (each ball dips the RPM, and
        // pausing for every dip is what made balls crawl in).
        firing = firing ? aim.keepFiring(turret) : aim.readyToFire(turret, shooter, hood);
        aim.setFiring(firing);
        if (firing) {
            if (aim.passing()) {
                // PASSING: precision doesn't matter, volume does. Feed as fast as it goes so the
                // balls leave in a tight stream ("Pass Feed Volts", saved, default 12 V).
                double v = aim.passFeedVolts();
                index.feed(v);
                uptake.feed(v);
            } else {
                index.feed(); // forward with automatic unjam
                uptake.feed();
            }
            SmartDashboard.putString("Shot Ready", "FIRING");
        } else {
            index.stop();
            uptake.stop();
            SmartDashboard.putString("Shot Ready", aim.readyText(turret, shooter, hood));
        }
    }

    @Override
    public void end(boolean interrupted) {
        aim.setFiring(false);
        index.stop();
        uptake.stop();
        if (spinShooter && stopShooterOnEnd) {
            shooter.stop();
        }
        SmartDashboard.putString("Shot Ready", "-");
    }
}
