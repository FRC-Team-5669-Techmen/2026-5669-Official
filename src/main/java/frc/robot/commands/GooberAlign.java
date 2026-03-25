package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Goober;
import frc.robot.subsystems.LimelightSubsystem;

public class GooberAlign extends Command {
    private final LimelightSubsystem limelight;
    private final Goober turret;
    
    // Start at 1 so it sweeps right if it starts without a tag
    private int seekDirection = 1; 
    
    // Safe speed for scanning back and forth
    private final double SEARCH_SPEED = 0.15; 

    public GooberAlign(LimelightSubsystem limelight, Goober turret) {
        this.limelight = limelight;
        this.turret = turret;
        addRequirements(turret, limelight);
    }

    @Override
    public void execute() {
        if (limelight.isTargetAvailable() && limelight.isValidTarget()) {
            // Track the target
            double tx = -limelight.getNewTX();
            turret.aimAtTarget(tx);
        } else {
            // Hunt for the target
            double currentPosition = turret.getPosition();

            // Wrap around safely inside the 10.0 and -25.0 hardware soft limits
            if (currentPosition >= 9.5) {
                seekDirection = -1; // Hit right limit, sweep left
            } else if (currentPosition <= -24.5) {
                seekDirection = 1;  // Hit left limit, sweep right
            }
 
            // Apply the sweep speed
           if (seekDirection == 1) {
                turret.setMotorSpeed(SEARCH_SPEED); 
            } else if (seekDirection == -1) {
                turret.setMotorSpeed(-SEARCH_SPEED);
            }
        }
    }

    @Override
    public boolean isFinished() {
       return !limelight.isValidTarget();
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
        seekDirection = 1; // Reset state for the next time the command runs
    }
}+2