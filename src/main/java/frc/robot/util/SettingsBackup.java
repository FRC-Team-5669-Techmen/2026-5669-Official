package frc.robot.util;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;
import java.util.Map;
import java.util.TreeMap;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * SETTINGS BACKUP (Oct 28): every value saved on the roboRIO (WPILib Preferences: recorded shot,
 * pass and climb tables, feed volts, turret far end, backup spots, saved switches) is copied to
 * /home/lvuser/shotmap/robot-settings.json whenever one changes (checked every 5 s, on its own
 * thread). It is served at http://10.56.69.2:5805/robot-settings.json and "Sync Match Data"
 * copies it into the repo, so a reimaged or wiped roboRIO never loses a tuning session, and the
 * numbers can be pasted into the code (ShotTable.defaults(), Constants).
 */
public final class SettingsBackup {
    private static final File kFile = new File(
        RobotBase.isReal() ? "/home/lvuser/shotmap" : "build/sim-shotmap", "robot-settings.json");
    private static final ObjectMapper kJson = new ObjectMapper().enable(SerializationFeature.INDENT_OUTPUT);
    private static String lastWritten = "";
    private static double nextCheck = 0;
    private static volatile boolean busy = false;

    private SettingsBackup() {}

    /** Call every loop; does real work at most every 5 s. */
    public static void periodic(double now, String codeVersion) {
        if (now < nextCheck || busy) return;
        nextCheck = now + 5.0;
        Map<String, Object> values = new TreeMap<>();
        NetworkTable table = NetworkTableInstance.getDefault().getTable("Preferences");
        for (String key : Preferences.getKeys()) {
            Object v = table.getEntry(key).getValue().getValue();
            if (v != null) values.put(key, v);
        }
        String body;
        try {
            body = kJson.writeValueAsString(values);
        } catch (Exception e) {
            return;
        }
        if (body.equals(lastWritten)) return;
        lastWritten = body;
        String stamped = "{\n  \"codeVersion\" : " + quote(codeVersion) + ",\n  \"savedAt\" : "
            + quote(java.time.LocalDateTime.now().withNano(0).toString()) + ",\n  \"preferences\" : " + body + "\n}\n";
        busy = true;
        Thread t = new Thread(() -> {
            try {
                kFile.getParentFile().mkdirs();
                File tmp = new File(kFile.getPath() + ".tmp");
                Files.writeString(tmp.toPath(), stamped);
                Files.move(tmp.toPath(), kFile.toPath(), StandardCopyOption.REPLACE_EXISTING);
            } catch (Exception e) {
                System.out.println("SETTINGS BACKUP | could not write: " + e.getMessage());
            } finally {
                busy = false;
            }
        }, "SettingsBackup");
        t.setDaemon(true);
        t.start();
    }

    private static String quote(String s) {
        try { return kJson.writeValueAsString(s); } catch (Exception e) { return "\"?\""; }
    }
}
