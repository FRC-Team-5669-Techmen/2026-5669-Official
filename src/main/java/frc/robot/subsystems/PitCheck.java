package frc.robot.subsystems;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

/**
 * PIT CHECK: the robot checks itself once a second, all the time (no button). Dashboard
 * "Pit Check" says ALL GO or lists what is wrong; "Pit Check Details" shows every item. Changes are
 * printed to the Driver Station console ("PIT CHECK |") and logged. Look at it before every match.
 *
 * <p>It only reads; it never moves anything.
 */
public class PitCheck extends SubsystemBase {
    private record Device(String name, ParentDevice device) {}

    private final List<Device> devices = new ArrayList<>();
    private final Goober turret;
    private final VisionLocalizer localizer;
    private int loops = 0;
    private double lastHeartbeat = -1;
    private int heartbeatStale = 0;
    private String lastSummary = "";

    public PitCheck(Goober turret, VisionLocalizer localizer) {
        this.turret = turret;
        this.localizer = localizer;
        String[] swerve = {"", "BR turn", "FR turn", "BL drive", "BR drive", "FL turn", "FL drive", "BL turn", "FR drive"};
        for (int id = 1; id <= 8; id++) devices.add(new Device("Talon " + id + " (" + swerve[id] + ")", new TalonFX(id)));
        String[] coders = {"FR", "FL", "BL", "BR"};
        for (int id = 9; id <= 12; id++) devices.add(new Device("CANcoder " + id + " (" + coders[id - 9] + ")", new CANcoder(id)));
        String[][] mech = {{"13", "Spindex"}, {"14", "Turret"}, {"15", "Uptake"}, {"16", "Shooter L"}, {"17", "Shooter R"},
            {"18", "Hood"}, {"19", "Elevator"}, {"20", "Intake pivot R"}, {"21", "Intake pivot L"}, {"22", "Intake roller"}};
        for (String[] m : mech) devices.add(new Device("Talon " + m[0] + " (" + m[1] + ")", new TalonFX(Integer.parseInt(m[0]))));
        devices.add(new Device("Pigeon 41", new Pigeon2(41)));
        SmartDashboard.putString("Pit Check", "starting...");
    }

    @Override
    public void periodic() {
        if (++loops % 50 != 0) return; // once a second
        List<String> problems = new ArrayList<>();
        List<String> lines = new ArrayList<>();

        // 1. Battery.
        double volts = RobotController.getBatteryVoltage();
        if (DriverStation.isDisabled() && volts < 12.4) problems.add(String.format("battery %.2f V: swap it", volts));
        lines.add(String.format("Battery %.2f V", volts));

        // 2. Every CAN device answering (only after 5 s, so boot-up doesn't count).
        if (loops > 250) {
            for (Device d : devices) {
                if (!d.device().isConnected(0.5)) problems.add(d.name() + " not on CAN");
            }
        }
        lines.add("CAN: " + devices.size() + " devices checked");

        // 3. Limelight alive (its heartbeat counter keeps changing).
        double hb = LimelightHelpers.getLimelightNTDouble("limelight", "hb");
        heartbeatStale = (hb == lastHeartbeat) ? heartbeatStale + 1 : 0;
        lastHeartbeat = hb;
        if (heartbeatStale >= 2) problems.add("Limelight not responding");
        lines.add("Limelight heartbeat " + (heartbeatStale >= 2 ? "STALE" : "OK"));

        // 4. Practice-field switch: must be OFF on a real field.
        boolean practice = LimelightSubsystem.practiceFieldMode();
        if (practice && DriverStation.isFMSAttached()) problems.add("Practice Field switch ON at competition: turn it OFF");
        lines.add("Practice Field: " + (practice ? "ON (hub tags only)" : "OFF (all tags)"));

        // 5. Alliance known (aiming picks the hub from it).
        var alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) problems.add("no alliance from the Driver Station");
        lines.add("Alliance: " + (alliance.isEmpty() ? "?" : alliance.get().toString()));

        // 6. Turret far end saved (aiming does nothing without it).
        if (!turret.endMeasured()) problems.add("turret far end not set");
        lines.add(String.format("Turret angle %.0f deg", turret.getAngleDegrees()));

        // 7. Position known.
        lines.add("Pose trusted: " + localizer.poseTrusted());

        // 8. Shot table: recorded points (saved on the roboRIO).
        String recorded = Preferences.getString("Shot Table Recorded", "");
        int points = recorded.isBlank() ? 0 : recorded.split(";").length;
        if (points < 4) problems.add("shot table has " + points + " recorded points (need 4+)");
        lines.add("Shot table recorded points: " + points);

        // 9. roboRIO disk: logs stop saving when it fills up.
        long freeMb = new File("/home/lvuser").getUsableSpace() / (1024 * 1024);
        if (freeMb > 0 && freeMb < 50) problems.add("roboRIO disk " + freeMb + " MB free: delete old logs");
        lines.add("roboRIO free disk: " + freeMb + " MB");

        String summary = problems.isEmpty() ? "ALL GO" : problems.size() + " problem(s): " + String.join(" | ", problems);
        SmartDashboard.putString("Pit Check", summary);
        SmartDashboard.putStringArray("Pit Check Details", lines.toArray(new String[0]));
        DogLog.log("PitCheck/Summary", summary);
        if (!summary.equals(lastSummary)) {
            System.out.println("PIT CHECK | " + summary);
            lastSummary = summary;
        }
    }
}
