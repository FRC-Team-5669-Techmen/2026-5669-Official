package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;

import java.util.Map;

public class DriverDashboard {
    
    // --- PRIMARY ZONE ENTRIES ---
    private final GenericEntry matchPhaseEntry;
    private final GenericEntry matchTimeEntry;
    private final GenericEntry voltageEntry;
    
    private final GenericEntry fmsConnectedEntry;
    private final GenericEntry visionLockEntry;

    // --- HUB SHIFT ENTRIES (Row 2) ---
    private final GenericEntry hubOpenEntry;
    private final GenericEntry shiftCountdownEntry;

    // --- MINIMIZED ZONE ENTRIES (Row 3) ---
    private final GenericEntry limelightTxEntry;
    private final GenericEntry limelightTyEntry;
    private final GenericEntry shooterRpmEntry; // NEW: RPM Indicator

    public DriverDashboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Match Dash");

        // ==========================================
        // PRIMARY ZONE (Top 75% - Rows 0 & 1)
        // ==========================================
        matchPhaseEntry = tab.add("Match Phase", "DISABLED")
            .withPosition(0, 0).withSize(3, 2).withWidget(BuiltInWidgets.kTextView).getEntry();

        matchTimeEntry = tab.add("Match Time", 150)
            .withPosition(3, 0).withSize(2, 2).withWidget(BuiltInWidgets.kDial)
            .withProperties(Map.of("min", 0, "max", 150)).getEntry();

        voltageEntry = tab.add("Voltage", 12.6)
            .withPosition(5, 0).withSize(2, 2).withWidget(BuiltInWidgets.kVoltageView).getEntry();

        fmsConnectedEntry = tab.add("FMS Connected", false)
            .withPosition(7, 0).withSize(2, 1).withWidget(BuiltInWidgets.kBooleanBox).getEntry();

        visionLockEntry = tab.add("Vision Lock", false)
            .withPosition(7, 1).withSize(2, 1).withWidget(BuiltInWidgets.kBooleanBox).getEntry();

        // ==========================================
        // HUB SHIFT ZONE (Row 2)
        // ==========================================
        hubOpenEntry = tab.add("HUB OPEN FOR US", false)
            .withPosition(0, 2).withSize(3, 1).withWidget(BuiltInWidgets.kBooleanBox).getEntry();

        shiftCountdownEntry = tab.add("Next Shift In (s)", 0.0)
            .withPosition(3, 2).withSize(2, 1).withWidget(BuiltInWidgets.kTextView).getEntry();

        // ==========================================
        // MINIMIZED ZONE (Bottom 25% - Row 3)
        // ==========================================
        limelightTxEntry = tab.add("Rizz tx", 0.0)
            .withPosition(0, 3).withSize(1, 1).getEntry();
            
        limelightTyEntry = tab.add("Rizz ty", 0.0)
            .withPosition(1, 3).withSize(1, 1).getEntry();

        // NEW: Shooter RPM tucked right next to the Limelight stats
        shooterRpmEntry = tab.add("Shooter RPM", 0.0)
            .withPosition(2, 3).withSize(2, 1).withWidget(BuiltInWidgets.kTextView).getEntry();
    }

    /**
     * Updates the dashboard with live data.
     */
    public void updateLiveStats(double tx, double ty, boolean hasVisionTarget, boolean isHubOpen, double currentRpm) {
        double matchTime = Math.max(0, DriverStation.getMatchTime());

        // Update Primary Stats
        matchTimeEntry.setDouble(matchTime);
        voltageEntry.setDouble(RobotController.getBatteryVoltage());
        fmsConnectedEntry.setBoolean(DriverStation.isFMSAttached());
        visionLockEntry.setBoolean(hasVisionTarget);

        // Update Hub Status
        hubOpenEntry.setBoolean(isHubOpen);

        // Calculate Shift Countdown
        double shiftTime = 0.0;
        if (matchTime > 130) {
            shiftTime = matchTime - 130; // Waiting for shifts to begin
        } else if (matchTime <= 130 && matchTime > 30) {
            shiftTime = (matchTime - 30) % 25; // Exact time remaining in the current 25s shift
        } else {
            shiftTime = 0.0; // Shifts are over (Endgame)
        }
        
        // Round to 1 decimal place so it doesn't flicker wildly with long decimals
        shiftCountdownEntry.setDouble(Math.round(shiftTime * 10.0) / 10.0);

        // Calculate Match Phase
        String phase = "DISABLED";
        if (DriverStation.isAutonomousEnabled()) phase = "AUTONOMOUS";
        else if (DriverStation.isTeleopEnabled()) phase = "TELEOP";
        else if (DriverStation.isTestEnabled()) phase = "TEST";
        matchPhaseEntry.setString(phase);

        // Update Minimized Stats
        limelightTxEntry.setDouble(tx);
        limelightTyEntry.setDouble(ty);
        
        // Update RPM (rounded to a clean whole number)
        shooterRpmEntry.setDouble(Math.round(currentRpm));
    }
}