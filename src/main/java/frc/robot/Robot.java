// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private double autoStart = 0;

    private String autoTime() {
        return String.format(java.util.Locale.US, " after %.1f s (auto is 20 s)", Timer.getFPGATimestamp() - autoStart);
    }

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();

        DogLog.setOptions(new DogLogOptions()
                .withLogExtras(true)
                .withCaptureDs(true)
                .withNtPublish(true)
                .withCaptureNt(true));
        DogLog.setPdh(new PowerDistribution());

        // roboRIO 2 cuts the motors at 6.75 V by default; 581 and 1678 run 5.5 V, 5000 6.0 V. 6.0 V
        // keeps the robot driving through a hard push or climb sag instead of going limp.
        edu.wpi.first.wpilibj.RobotController.setBrownoutVoltage(6.0);

        // WHY DID IT STOP? Every interrupted command is printed (console "CMD |") and logged with what
        // interrupted it, so after a match anyone can see why the auto or the aim ended.
        CommandScheduler.getInstance().onCommandInterrupt((cmd, by) -> {
            String who = by.map(Command::getName).orElse("disable / mode change / cancel");
            String line = cmd.getName() + " interrupted by " + who;
            if (cmd == m_autonomousCommand) line = "AUTO " + line + autoTime();
            System.out.println("CMD | " + line);
            frc.robot.subsystems.PracticeMatch.note(line);
            DogLog.log("Commands/Interrupted", line);
        });
        CommandScheduler.getInstance().onCommandFinish(cmd -> {
            if (cmd == m_autonomousCommand) {
                System.out.println("CMD | AUTO finished" + autoTime());
                frc.robot.subsystems.PracticeMatch.note("AUTO finished" + autoTime());
                DogLog.log("Commands/AutoSeconds", Timer.getFPGATimestamp() - autoStart);
            }
        });

        // Elastic "Load Layout From Robot" (Ctrl+D) downloads src/main/deploy/elastic-layout.json from here.
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
        // Shot Map Day: the shot log, in any browser while connected: http://10.56.69.2:5805/shot-log.csv
        new java.io.File("/home/lvuser/shotmap").mkdirs();
        WebServer.start(5805, "/home/lvuser/shotmap");
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        HubShiftTracker.update(); // match clock (own clock in a practice match)
        CommandScheduler.getInstance().run();

        m_robotContainer.updateDashboard();
        updateDriverRumble();
    }

    /**
     * Haptic feedback for the driver during TELEOP:
     * <ul>
     *   <li>Strong pulses during the last 5 seconds of each SHIFT, warning that
     *       the HUBs are about to swap.</li>
     *   <li>Gentle constant rumble while our HUB is open for scoring.</li>
     *   <li>Silence otherwise (and in every other mode).</li>
     * </ul>
     */
    private void updateDriverRumble() {
        var driverHid = m_robotContainer.driverController.getHID();

        if (!DriverStation.isTeleopEnabled()) {
            driverHid.setRumble(RumbleType.kBothRumble, 0.0);
            return;
        }

        double matchTime = HubShiftTracker.matchTime();
        double rumble = 0.0;

        // NO GAME DATA in a real match (FMS attached): the FMS sends it about 3 s after auto ends,
        // so it is missing for the first seconds of teleop BY DESIGN. Still missing 6 s into teleop
        // = it isn't coming: buzz BOTH controllers (pulses, for up to 14 s or until "Auto Winner" is
        // set on the dashboard).
        if (DriverStation.isFMSAttached() && HubShiftTracker.gameDataMissing() && matchTime < 134 && matchTime > 120) {
            double r = (Timer.getFPGATimestamp() % 1.0) < 0.3 ? 1.0 : 0.0;
            driverHid.setRumble(RumbleType.kBothRumble, r);
            m_robotContainer.operatorController.getHID().setRumble(RumbleType.kBothRumble, r);
            return;
        }

        // DEFENSE warnings come first: pin count near the 3 s limit = strong fast pulses; last
        // 30 s next to THEIR tower = steady strong rumble (see DefenseMode).
        var defense = m_robotContainer.defense;
        if (defense.pinWarning()) {
            driverHid.setRumble(RumbleType.kBothRumble, (Timer.getFPGATimestamp() % 0.25) < 0.15 ? 1.0 : 0.0);
            return;
        }
        // Mode-change patterns (Haptics: 1 short = on, 2 short = off, long = vacuum/dump on).
        double pattern = frc.robot.controls.Haptics.level(frc.robot.controls.Haptics.Pad.DRIVER);
        if (pattern >= 0) {
            driverHid.setRumble(RumbleType.kBothRumble, pattern);
            return;
        }
        if (defense.changeBuzz()) {
            driverHid.setRumble(RumbleType.kBothRumble, 0.8);
            return;
        }
        if (defense.towerWarning()) {
            driverHid.setRumble(RumbleType.kBothRumble, 0.6);
            return;
        }

        // SHIFT-change countdown: shifts run 2:10 -> 0:30 in 25 s windows
        if (matchTime <= 130 && matchTime > 30) {
            double timeInShift = (matchTime - 30) % 25;
            if (timeInShift <= 5.0 && timeInShift > 0.1 && (timeInShift % 1.0) < 0.2) {
                rumble = 0.9;
            }
        }

        // Constant low rumble while our HUB is open (unless we're mid-pulse)
        if (rumble == 0.0 && matchTime > 0 && m_robotContainer.isHubOpenForUs()) {
            rumble = 0.1;
        }

        driverHid.setRumble(RumbleType.kBothRumble, rumble);
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_robotContainer.onAutoStart();
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        autoStart = Timer.getFPGATimestamp();
        System.out.println("CMD | AUTO start: " + m_robotContainer.selectedAutoName());

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
