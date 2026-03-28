// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        
        // --> DASHBOARD CONTINUOUSLY UPDATES HERE <--
        m_robotContainer.updateDashboard();

        boolean ready = m_robotContainer.isHubOpenForUs();
        SmartDashboard.putBoolean("HUB OPEN", ready);
        double matchTime = edu.wpi.first.wpilibj.DriverStation.getMatchTime();
        var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
        boolean isOurTurn = m_robotContainer.isHubOpenForUs();
        boolean isTeleop = edu.wpi.first.wpilibj.DriverStation.isTeleop();

        if(isTeleop) {
            if (isOurTurn && matchTime > 0) {
            // Constant vibration while it's our turn to score
            m_robotContainer.driverController.getHID().setRumble(edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0.1);
            } 
        }
        // coundown
        else if (matchTime <= 130 && matchTime > 30) {
            double timeInShift = (matchTime - 30) % 25;
            
            if (timeInShift <= 5.0 && timeInShift > 0.1) {
                if ((timeInShift % 1.0) < 0.2) {
                    m_robotContainer.driverController.getHID().setRumble(
                        edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0.9);
                } else {
                    m_robotContainer.driverController.getHID().setRumble(
                        edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0);
                }
            } else {
                m_robotContainer.driverController.getHID().setRumble(
                    edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0);
            }
        } 
        else {
            // Stop all vibration if it's not our turn and not a countdown window
            m_robotContainer.driverController.getHID().setRumble(
                edu.wpi.first.wpilibj.GenericHID.RumbleType.kBothRumble, 0);
        }
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