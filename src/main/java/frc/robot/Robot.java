// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

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
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
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

        double matchTime = DriverStation.getMatchTime();
        double rumble = 0.0;

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
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

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
