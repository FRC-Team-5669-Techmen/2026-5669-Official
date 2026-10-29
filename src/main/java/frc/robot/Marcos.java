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

        // GET READY TO SCORE while still driving home (Oct 29): turret on OUR HUB, shooter at the
        // scoring speed for the distance, hood to its angle as soon as the trench guard lets it
        // up. No feeding. Event marker ZONE on a path that ends at a shooting spot, so the
        // "visionScore" after it fires at once instead of first spinning up and turning the
        // turret. Always our hub (never a pass corner), so the zone can start in the neutral zone.
        NamedCommands.registerCommand("prepareShot", prepareShot(aim, shooter, goober, gooba, drivetrain));

        // Turret keeps the Limelight on our hub (camera position fixes while driving), nothing
        // else moves. Event marker zone for paths without a shot at the end. (An auto holds every
        // motor its commands use for its whole run, so the turret's own "watch the hub in autos"
        // default never runs in an auto that scores.)
        NamedCommands.registerCommand("watchHub", Commands.run(() -> aimTurretAtHub(aim, goober, drivetrain), goober)
            .finallyDo(() -> { goober.stop(); goober.clearAngleTarget(); })
            .withName("watchHub"));

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
        // Roller in for the REST OF AUTO, scheduled on its own (not part of the auto's sequence), so
        // it keeps spinning through the paths and stops by itself when auto ends. It steps aside
        // while balls are being fired: then the Agitator runs the roller exactly like in teleop
        // (5 V assist + tap-outs), because 12 V while feeding packs the hopper and jams the inlet.
        // No requirements of its own (like VacuumCommand), and none of these named commands
        // require the roller, so scheduling it can never cancel the auto that started it.
        Command rollerIn = intakeRoller.runCommand().withName("Auto: intake roller");
        Command rollerForAuto = Commands.run(() -> {
                if (aim.isFiring()) {
                    if (rollerIn.isScheduled()) rollerIn.cancel();
                } else if (!rollerIn.isScheduled()) {
                    CommandScheduler.getInstance().schedule(rollerIn);
                }
            })
            .onlyWhile(DriverStation::isAutonomousEnabled)
            .finallyDo(rollerIn::cancel)
            .withName("Auto: roller manager");
        // One shared instance: scheduling it again while it runs does nothing.
        Runnable startRoller = () -> CommandScheduler.getInstance().schedule(rollerForAuto);
        // Intake DOWN + roller on: the very same move as Driver A's DOWN press (deployIfUp runs the
        // same Motion Magic downMove), so the same speed and strength, and it stops pushing once
        // it reaches the floor. Does nothing if the code already knows it's down.
        Runnable lower = () -> {
            groundIntakePivot.deployIfUp();
            startRoller.run();
        };
        Runnable raise = () -> {
            rollerForAuto.cancel();
            groundIntakePivot.stowIfDown();
        };

        // Instant: the path keeps going while the intake comes down. Use it as the auto's first
        // step, or as an event marker once the robot is clear of the trench and bump.
        NamedCommands.registerCommand("lowerIntake", Commands.runOnce(lower));
        // Same thing under a clearer name, for new autos.
        NamedCommands.registerCommand("intakeDown", Commands.runOnce(lower));
        // Oct 29: starting a match with the intake down is NOT legal (starting configuration), so
        // the robot always starts with it UP. This name only stays so older auto files load, and
        // it now does exactly what lowerIntake does. (It used to set the pivot's zero at the
        // floor without moving it: with the intake really UP that zero would be wrong all match.
        // That job is the "Zero Intake (intake is DOWN)" dashboard button now, for the shop.)
        NamedCommands.registerCommand("intakeStartsDown", Commands.runOnce(lower));
        // Intake UP + roller off.
        NamedCommands.registerCommand("raiseIntake", Commands.runOnce(raise));
        // Intake down + roller on, then 2 s to let it pick up. The roller keeps running for the rest
        // of auto like lowerIntake (it used to require the roller here, which would have let the
        // roller manager cancel the whole auto).
        NamedCommands.registerCommand("intake", Commands.runOnce(lower).andThen(Commands.waitSeconds(2.0)));
        // Roller on for the rest of auto (the intake stays where it is).
        NamedCommands.registerCommand("runGroundIntake", Commands.runOnce(startRoller));

        // Event markers that only have a NAME (no command attached, the PathPlanner default) just
        // fire a trigger with that name. These make the intake names work that way too.
        new EventTrigger("runGroundIntake").onTrue(Commands.runOnce(startRoller));
        new EventTrigger("lowerIntake").onTrue(Commands.runOnce(lower));
        new EventTrigger("intakeDown").onTrue(Commands.runOnce(lower));
        new EventTrigger("intakeStartsDown").onTrue(Commands.runOnce(lower));
        new EventTrigger("raiseIntake").onTrue(Commands.runOnce(raise));

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

    /** Point the turret at OUR HUB from the field position (no shoot-on-the-move lead). */
    private static void aimTurretAtHub(AimSubsystem aim, Goober goober, CommandSwerveDrivetrain drivetrain) {
        if (!aim.poseUsable()) {
            goober.stop();
            return;
        }
        var s = aim.hubSolution();
        goober.aimAtAngleWithRate(s.turretDegrees(), s.reachable() ? s.turretRateDegPerSec()
            : Math.toDegrees(drivetrain.getState().Speeds.omegaRadiansPerSecond));
    }

    /** The "prepareShot" named command: turret, shooter and hood ready to score at our hub, no feeding. */
    public static Command prepareShot(AimSubsystem aim, ShooterSubsystem shooter, Goober goober,
            GoobaSubsystem gooba, CommandSwerveDrivetrain drivetrain) {
        return Commands.run(() -> {
            aimTurretAtHub(aim, goober, drivetrain);
            if (!aim.poseUsable()) return;
            double distance = aim.hubSolution().distanceMeters();
            shooter.runAtRPM(aim.tableRpm(distance));
            // The hood holds DOWN by itself near a trench (GoobaSubsystem trench guard).
            if (Constants.Gooba.kHoodEnabled) gooba.setPosition(aim.tableHood(distance));
        }, goober, shooter, gooba)
            // Turret stops where it is pointing; the wheel keeps spinning for the score after it.
            .finallyDo(() -> { goober.stop(); goober.clearAngleTarget(); })
            .withName("prepareShot");
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
                // Stopped next to a trench (the 3-piece shooting spots are ~0.15 m from the trench
                // guard): let the hood up while the robot stays put. AimSubsystem refuses it if
                // the hood is really under the bar, and it ends once the robot moves 0.15 m.
                aim.engageTrenchOverride();
                firstFire[0] = Double.NaN;
                lastBall[0] = Double.NEGATIVE_INFINITY;
                shotsAtStart[0] = shooter.shotsStarted();
            })
            .withTimeout(3.5)
            .withName("score (until empty)");
    }
}
