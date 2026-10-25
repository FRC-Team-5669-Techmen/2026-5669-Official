package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.aiming.ShotSolver;
import frc.robot.subsystems.AimSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GoobaSubsystem;
import frc.robot.subsystems.Goober;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * SHOOT FROM ANYWHERE aiming (Operator RB toggles it; autos use it too). Keeps the turret on our
 * hub and the hood at the shot table's angle for the distance, from the robot's field position,
 * whether or not a hub tag is in view.
 *
 * <p>If the robot's position isn't known yet (no vision fix, gyro never reset), it falls back to
 * last season's way: turret follows OUR hub tag's tx, hood follows its ty. With no hub tag in view
 * it holds still (no sweeping: the position-based aim already knows where the hub is).
 *
 * <p>If the hub is behind the turret (the ~90 degree dead zone), the turret waits at the nearest
 * end and the drive code turns the robot (see RobotContainer heading assist).
 */
public class AimCommand extends Command {
    private final AimSubsystem aim;
    private final Goober turret;
    private final GoobaSubsystem hood;
    private final LimelightSubsystem limelight;
    private final CommandSwerveDrivetrain drivetrain;
    private final LinearFilter tyFilter = LinearFilter.movingAverage(5);

    public AimCommand(AimSubsystem aim, Goober turret, GoobaSubsystem hood,
                      LimelightSubsystem limelight, CommandSwerveDrivetrain drivetrain) {
        this.aim = aim;
        this.turret = turret;
        this.hood = hood;
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        addRequirements(turret, hood);
    }

    @Override
    public void initialize() {
        aim.setActive(true);
        turret.clearAngleTarget();
        turret.resetAimFilter();
        tyFilter.reset();
    }

    @Override
    public void execute() {
        if (aim.poseUsable()) {
            ShotSolver.Solution s = aim.solution();
            double spin = drivetrain.getState().Speeds.omegaRadiansPerSecond;
            turret.aimAtAngle(s.turretDegrees(), spin);
            if (!s.reachable()) {
                turret.setAimStatus("HUB BEHIND: turning robot");
            }
            if (Constants.Gooba.kHoodEnabled) {
                hood.setPosition(aim.targetHood());
            }
            SmartDashboard.putString("Aim Mode", "POSITION");
            return;
        }

        // Fallback: chase our hub tag like before.
        SmartDashboard.putString("Aim Mode", "TAG ONLY (position unknown)");
        turret.clearAngleTarget();
        if (limelight.isValidTarget() && turret.endMeasured()) {
            turret.aimAtTarget(limelight.getTX());
            if (Constants.Gooba.kHoodEnabled) {
                hood.setPosition(hood.getRotationForTy(tyFilter.calculate(limelight.getTY())));
            }
        } else {
            turret.stop();
            turret.setAimStatus("NO POSITION, NO HUB TAG");
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        aim.setActive(false);
        turret.stop();
        turret.clearAngleTarget();
        turret.setAimStatus("OFF");
        SmartDashboard.putString("Aim Mode", "OFF");
    }
}
