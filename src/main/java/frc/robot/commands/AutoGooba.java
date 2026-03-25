package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GoobaSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoGooba extends Command {
    private final GoobaSubsystem m_gooba;
    private final LimelightSubsystem m_vision;

    public AutoGooba(GoobaSubsystem gooba, LimelightSubsystem vision) {
        this.m_gooba = gooba;
        this.m_vision = vision;
        
        // CRITICAL FIX 1: The command must "own" the gooba subsystem to move it.
        // Do NOT add m_vision to the requirements here. If you do, it will 
        // crash and conflict with your GooberAlign turret tracking!
        addRequirements(m_gooba); 
    }


    @Override
    public void execute() {
        // We only want to interpolate and move if the Limelight actually sees a target.
        if (m_vision.isTargetAvailable()) {
            
            // 1. Get the offset 'tx' from the Limelight. 
            // Because the Limelight is on its side, 'tx' represents the real-world vertical angle!
            double currentTx = m_vision.getTX();
            
            // 2. Ask the Gooba subsystem for the correct hood rotation based on the 'tx' angle
            double targetRotations = m_gooba.getRotationValueFromTx(currentTx);
            
            // --- CALIBRATION PRINTOUT ---
            // Prints both the TX and the physical Hood position side-by-side
            System.out.println("[CALIBRATION] TX: " + currentTx + "  |  Hood Pos: " + m_gooba.getPosition());
            
            // 3. Move the hood
            m_gooba.setPosition(targetRotations);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Nothing needed here
    }

    @Override
    public boolean isFinished() {
        // CRITICAL FIX 3: This MUST return false. 
        // If it returns true, the command runs exactly once and stops forever.
        return false; 
    }
}