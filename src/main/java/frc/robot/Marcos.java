package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * Registers every PathPlanner NamedCommand and event trigger.
 * Must run BEFORE AutoBuilder.buildAutoChooser() — RobotContainer calls this first.
 *
 * <p>The registered string names are referenced by the .auto/.path files in
 * src/main/deploy/pathplanner — do not rename them without updating the autos.
 */
public class Marcos {
    public static void registerNamedCommands(
        ShooterSubsystem shooter,
        IndexSubsystem index,
        ShooterIntakeSubsystem shooterIntake,
        Goober goober,
        LimelightSubsystem limelight,
        GoobaSubsystem gooba,
        AimSubsystem aim,
        CommandSwerveDrivetrain drivetrain,
        GroundIntakePivotSubsystem groundIntakePivot,
        IntakeRollerSubsystem intakeRoller
    ) {
        NamedCommands.registerCommand("spinUpShooter",
            new RunShooterCommand(shooter, Constants.Shooter.kFastTargetRPM)
                .withTimeout(1.5)
        );

        NamedCommands.registerCommand("spinUpShooterSlow",
            new RunShooterCommand(shooter, Constants.Shooter.kSlowTargetRPM)
                .withTimeout(1.5)
        );

        // SHOOT FROM ANYWHERE (Oct 26): aim from the field position (the path sets it) and fire
        // every ball the moment the shot is ready, for up to 3.5 s. Works stopped or moving.
        NamedCommands.registerCommand("score", scoreUntilEmpty(aim, shooter, index, shooterIntake, goober, gooba, limelight, drivetrain));

        // Same as "score" (Oct 27). It used to spin to a fixed 5400 RPM and feed with the turret
        // wherever it was: with the turret at HOME (shooter facing the robot front) the Center
        // Preload auto faced our own wall. Aiming from the field position needs no camera: the
        // auto's starting pose is enough.
        NamedCommands.registerCommand("scorePreload", scoreUntilEmpty(aim, shooter, index, shooterIntake, goober, gooba, limelight, drivetrain));

        // GET READY TO SHOOT while still driving (Oct 27): turret + hood on the hub and the
        // shooter at the table speed, no feeding. Put it on the last part of a path that ends at
        // a shooting spot (event marker zone in PathPlanner), so the "score" right after it fires
        // at once instead of spending its first second spinning up and turning the turret.
        NamedCommands.registerCommand("prepareShot",
            Commands.parallel(
                new AimCommand(aim, goober, gooba, limelight, drivetrain),
                Commands.run(() -> shooter.runAtRPM(aim.targetRpm()), shooter)
            )
        );

        // Timeout prevents auto hangs
        NamedCommands.registerCommand("shoot",
            new FuelHandlingCommand(index, shooterIntake, shooter, true).withTimeout(2.0)
        );

        NamedCommands.registerCommand("stopShooter",
            new InstantCommand(shooter::stop, shooter)
        );

        // Old "intake": runs the spindexer, uptake AND SHOOTER BACKWARDS (last season's intake went
        // through the shooter). Kept under an honest name; no auto uses it.
        NamedCommands.registerCommand("reverseFeed",
            new FuelHandlingCommand(index, shooterIntake, shooter, false).withTimeout(2.0)
        );

        // Same as "score" (kept so existing autos load): aim from position, fire when ready.
        NamedCommands.registerCommand("visionScore", scoreUntilEmpty(aim, shooter, index, shooterIntake, goober, gooba, limelight, drivetrain));

        // Point the turret (and hood) at the hub from the field position, for 2 s.
        NamedCommands.registerCommand("aimTurret",
            new AimCommand(aim, goober, gooba, limelight, drivetrain).withTimeout(2.0)
        );
        NamedCommands.registerCommand("alignTurret",
            new AimCommand(aim, goober, gooba, limelight, drivetrain).withTimeout(2.0)
        );

        // ==========================================
        // GROUND INTAKE (Oct 27): pivot 20/21 + roller 22. These were no-ops since the
        // pneumatics came off, so the 3-piece autos drove through the balls without picking
        // any up.
        // ==========================================
        // Roller in for the REST OF AUTO: scheduled on its own (not part of the auto's sequence),
        // so it keeps spinning through the paths and shots, and stops by itself when auto ends.
        // One shared instance: scheduling it again while it runs does nothing.
        Command rollerForAuto = intakeRoller.runCommand()
            .onlyWhile(DriverStation::isAutonomousEnabled)
            .withName("Auto: intake roller");
        // A command can only be used in one place, so each use below makes its own runOnce.
        Runnable startRoller = () -> CommandScheduler.getInstance().schedule(rollerForAuto);

        // Intake DOWN + roller on (for the rest of auto). Instant: the path starts right away
        // while the intake goes down. First step of both 3-piece autos.
        NamedCommands.registerCommand("lowerIntake",
            Commands.runOnce(() -> {
                groundIntakePivot.deployIfUp();
                startRoller.run();
            }));
        // Same thing under a clearer name, for new autos.
        NamedCommands.registerCommand("intakeDown",
            Commands.runOnce(() -> {
                groundIntakePivot.deployIfUp();
                startRoller.run();
            }));
        // Intake UP + roller off.
        NamedCommands.registerCommand("raiseIntake",
            Commands.runOnce(() -> {
                rollerForAuto.cancel();
                groundIntakePivot.stowIfDown();
            }));
        // Intake down + roller for 2 s (stays down after).
        NamedCommands.registerCommand("intake",
            Commands.runOnce(groundIntakePivot::deployIfUp)
                .andThen(intakeRoller.runCommand().withTimeout(2.0)));

        // The "runGroundIntake" event markers in the pickup paths have no command attached, so
        // they fire this trigger: roller on for the rest of auto (in case lowerIntake was removed).
        NamedCommands.registerCommand("runGroundIntake", Commands.runOnce(startRoller));
        new EventTrigger("runGroundIntake").onTrue(Commands.runOnce(startRoller));

        // Pneumatics are gone: still no-ops so old autos load cleanly.
        NamedCommands.registerCommand("togglePiston1", Commands.none());
        NamedCommands.registerCommand("togglePiston2", Commands.none());

        // ==========================================
        // HOOD (back on Talon FX 18, Oct 2026)
        // ==========================================
        NamedCommands.registerCommand("deployGooba", new GoobaToggleCommand(gooba, true));
        NamedCommands.registerCommand("stowGooba", new GoobaToggleCommand(gooba, false));
        NamedCommands.registerCommand("autoAimGooba", new AutoGooba(gooba, limelight).withTimeout(2.0));
    }

    /**
     * AUTO SCORE that ends as soon as the hopper is EMPTY (Oct 27): once firing has started, no
     * ball has left for kEmptySeconds = done, so the auto moves on instead of waiting out the full
     * 3.5 s (the 3-piece autos ran ~23 s of a 20 s auto). Still stops at 3.5 s no matter what.
     */
    private static final double kEmptySeconds = 1.0; // > the agitation pump trigger (0.8 s), so a bridged hopper gets one pump first

    public static Command scoreUntilEmpty(AimSubsystem aim, ShooterSubsystem shooter, IndexSubsystem index,
            ShooterIntakeSubsystem shooterIntake, Goober goober, GoobaSubsystem gooba,
            LimelightSubsystem limelight, CommandSwerveDrivetrain drivetrain) {
        double[] firstFire = {Double.NaN};
        double[] lastBall = {Double.NEGATIVE_INFINITY};
        int[] shotsAtStart = {0};
        return new ShootCommand(aim, shooter, index, shooterIntake, goober, gooba, true)
            .deadlineFor(new AimCommand(aim, goober, gooba, limelight, drivetrain))
            .until(() -> {
                double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
                if (Double.isNaN(firstFire[0]) && aim.isFiring()) firstFire[0] = now;
                if (Double.isNaN(firstFire[0])) return false;
                if (shooter.shotsStarted() > shotsAtStart[0]) lastBall[0] = shooter.lastShotTime();
                double since = now - Math.max(firstFire[0], lastBall[0]);
                return since > kEmptySeconds;
            })
            .beforeStarting(() -> {
                firstFire[0] = Double.NaN;
                lastBall[0] = Double.NEGATIVE_INFINITY;
                shotsAtStart[0] = shooter.shotsStarted();
            })
            .withTimeout(3.5)
            .withName("score (until empty)");
    }
}
