package frc.robot.subsystems;

import java.io.File;
import java.io.FileWriter;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.Locale;
import java.util.Random;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.function.Supplier;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HubShiftTracker;
import frc.robot.practice.MatchPhase;

/**
 * PRACTICE MATCH (Oct 28): full timed practice matches without an FMS.
 *
 * <p>How to run one: Driver Station <b>Practice</b> mode (Setup tab timing: Autonomous 20,
 * Teleop 140), team station Red. The robot runs the selected auto, then teleop, with the real
 * match clock. At teleop start (no FMS, no game data) the robot picks the AUTO WINNER AT RANDOM, so
 * the shift rumbles, the hub-closed hold and the "Our Hub" dashboard all behave like a real match.
 * Dashboard switch "Practice: Random Auto Winner" (saved, ON; never used with an FMS attached).
 *
 * <p>The MATCH SCREEN for a second device (laptop, TV, phone on the robot network):
 * <b>http://10.56.69.2:5806/</b> (over USB: http://172.22.11.2:5806/). The robot serves the page
 * (src/main/deploy/practice-match/index.html) and a live status file (match.json, 10 times a
 * second) from RAM. The page shows phase, clock, both hub lights, the auto winner, field-style
 * sounds and the scorecard, and keeps every match in a history table.
 *
 * <p>SCORECARD per match (from the flywheel ball counter): balls launched in auto, into our hub
 * while it would be OPEN when the ball lands, while CLOSED (wasted), and passes; first climb lift
 * and hang times. Each match is appended to /home/lvuser/shotmap/practice-matches.csv
 * (http://10.56.69.2:5805/practice-matches.csv).
 */
public class PracticeMatch extends SubsystemBase {
    private static final String kRandomKey = "Practice: Random Auto Winner";
    private static final double kFlightGuess = 1.0; // s: hub state judged when the ball lands

    private final ShooterSubsystem shooter;
    private final AimSubsystem aim;
    private CommandSwerveDrivetrain drivetrain;
    private java.util.function.BooleanSupplier rtHeld = () -> false;
    private Supplier<String> autoName = () -> "";
    private frc.robot.practice.MatchTelemetry telemetry = null;
    private double matchStart = 0;
    private int lastFinished = 0;
    private String kind = "";
    private static final java.util.concurrent.ConcurrentLinkedQueue<String> notes = new java.util.concurrent.ConcurrentLinkedQueue<>();
    private static final String kMatchDir = "/home/lvuser/shotmap/matches";
    private static final String[] kCrew = {"Driver", "Operator", "Coach", "Battery"};
    private boolean logServer = false;
    private int serverTries = 0;
    private double autoOverAt = -1;

    /** A line for the current match's timeline (command interrupts, auto end...). Any thread. */
    public static void note(String text) { if (notes.size() < 200) notes.add(text); }

    /** Extra sources for the match telemetry (RobotContainer calls this once). */
    public void setSources(CommandSwerveDrivetrain drivetrain, java.util.function.BooleanSupplier operatorRt,
                           Supplier<String> autoName) {
        this.drivetrain = drivetrain;
        this.rtHeld = operatorRt;
        this.autoName = autoName;
    }
    private final Supplier<HubShiftTracker.AutoWinner> chooserPick;
    private final Random random = new Random();
    private final ExecutorService writer = Executors.newSingleThreadExecutor(r -> {
        Thread t = new Thread(r, "practice-match");
        t.setDaemon(true);
        t.setPriority(Thread.MIN_PRIORITY);
        return t;
    });

    private String dir = null; // RAM folder served on 5806
    private boolean wasAuto = false, wasTeleop = false;
    private boolean randomized = false;
    private String randomWinner = "";
    private int matchNumber = 0;
    private boolean inMatch = false;
    private int lastShots = 0;
    private int autoBalls, openBalls, closedBalls, passBalls;
    private double teleopStart = -1, liftTime = -1, hangTime = -1;
    private String lastResult = "";
    private int loops = 0;

    public PracticeMatch(ShooterSubsystem shooter, AimSubsystem aim, Supplier<HubShiftTracker.AutoWinner> chooserPick) {
        this.shooter = shooter;
        this.aim = aim;
        this.chooserPick = chooserPick;
        Preferences.initBoolean(kRandomKey, true);
        SmartDashboard.putBoolean(kRandomKey, Preferences.getBoolean(kRandomKey, true));
        SmartDashboard.putString("Practice Match", "Driver Station Practice mode, then the match screen at :5806");
        // Who's driving and which battery: typed on the Match tab, kept between matches (saved),
        // written into every match file so the app can tie matches to drivers and batteries.
        for (String k : kCrew) {
            String key = "Match " + k;
            Preferences.initString(key, "");
            SmartDashboard.putString(key, Preferences.getString(key, ""));
        }
        startServer();
    }

    /** Copy the page into a RAM folder and serve it with the live status on port 5806. */
    private void startServer() {
        try {
            String base = new File("/var/volatile/tmp").isDirectory() ? "/var/volatile/tmp" : System.getProperty("java.io.tmpdir");
            File d = new File(base, "practice-match");
            d.mkdirs();
            File page = new File(Filesystem.getDeployDirectory(), "practice-match/index.html");
            if (page.exists()) {
                Files.copy(page.toPath(), new File(d, "index.html").toPath(), StandardCopyOption.REPLACE_EXISTING);
            }
            dir = d.getPath();
            WebServer.start(5806, dir);
        } catch (Exception e) {
            System.out.println("PRACTICE | match screen not started: " + e.getMessage());
        }
    }

    private static boolean red() {
        return DriverStation.getAlliance().map(a -> a == Alliance.Red).orElse(true);
    }

    @Override
    public void periodic() {
        if (!logServer && ++serverTries % 250 == 1) {
            File d = new File(logDir());
            if (d.isDirectory()) {
                try { WebServer.start(5807, d.getPath()); logServer = true; } catch (Exception e) { /* retried */ }
            }
        }
        boolean auto = DriverStation.isAutonomousEnabled();
        boolean teleop = DriverStation.isTeleopEnabled();
        double now = Timer.getFPGATimestamp();
        double matchTime = HubShiftTracker.matchTime();
        boolean fms = DriverStation.isFMSAttached();

        // ---- match start / end ----
        if (auto && !wasAuto) startMatch(fms ? kindFromFms() : "auto_test");
        if (teleop && !wasTeleop) {
            if (!inMatch) startMatch("teleop_session"); // teleop-only practice still gets a scorecard
            else if (!fms && HubShiftTracker.practiceSequence()) kind = "practice";
            teleopStart = now;
            // Only a real match sequence (auto, then teleop within a few seconds: Driver Station
            // Practice mode) gets a random winner. Plain teleop (shot plotting, drive tests) never
            // does, so the hub-closed hold can't block shots there.
            if (HubShiftTracker.practiceSequence()) pickRandomWinner(fms);
        }
        if (!auto && !teleop && (wasAuto || wasTeleop)) {
            if (inMatch && wasTeleop) finishMatch();      // teleop over (or disabled): write the match
            else if (inMatch && wasAuto) autoOverAt = now; // auto over: teleop may follow (3-4 s on a field)
            if (randomized) {                       // back to the dashboard's choice
                HubShiftTracker.setAutoWinnerOverride(chooserPick.get());
                randomized = false;
            }
        }
        // Auto ran and no teleop followed within 8 s: it was an auto test, write it.
        if (inMatch && !auto && !teleop && autoOverAt > 0 && now - autoOverAt > 8) finishMatch();
        if (auto || teleop) autoOverAt = -1;
        wasAuto = auto;
        wasTeleop = teleop;

        // ---- scorecard ----
        String phaseName = MatchPhase.phase(auto, teleop, matchTime).name();
        int shots = shooter.shotsStarted();
        if (shots < lastShots) lastShots = shots; // stats were reset
        while (lastShots < shots) {
            lastShots++;
            if (!inMatch) continue;
            String cat;
            if (auto) { autoBalls++; cat = "auto"; }
            else if (aim.passing()) { passBalls++; cat = "pass"; }
            else if (HubShiftTracker.ourHubStateAt(kFlightGuess) == HubShiftTracker.HubState.CLOSED) { closedBalls++; cat = "closed"; }
            else { openBalls++; cat = "open"; }
            if (telemetry != null) telemetry.shot(phaseName, cat);
        }
        int fin = shooter.finishedShots();
        if (fin < lastFinished) lastFinished = fin;
        if (fin > lastFinished) {
            lastFinished = fin;
            if (telemetry != null) telemetry.launch(shooter.lastLaunchRpm(), shooter.lastShotTargetRpm());
        }
        if (telemetry != null && (auto || teleop)) sampleTelemetry(now, matchTime, phaseName);
        String n;
        while ((n = notes.poll()) != null) {
            if (telemetry != null) telemetry.event(now - matchStart, matchTime, phaseName, n);
        }
        if (inMatch && teleop) {
            if (liftTime < 0 && ElevatorSubsystem.isRaised()) liftTime = now - teleopStart;
            if (hangTime < 0 && ElevatorSubsystem.isHanging()) hangTime = now - teleopStart;
        }

        // ---- status for the match screen (10 times a second) ----
        if (++loops % 5 == 0 && dir != null) writeStatus(auto, teleop, matchTime, fms);
    }

    private static String kindFromFms() {
        return switch (DriverStation.getMatchType()) {
            case Qualification -> "qual";
            case Elimination -> "playoff";
            case Practice -> "practice_fms";
            default -> "fms";
        };
    }

    private void sampleTelemetry(double now, double matchTime, String phaseName) {
        double amps = 0, speed = 0;
        if (drivetrain != null) {
            for (var m : drivetrain.getModules()) amps += Math.abs(m.getDriveMotor().getSupplyCurrent().getValueAsDouble());
            var sp = drivetrain.getState().Speeds;
            speed = Math.hypot(sp.vxMetersPerSecond, sp.vyMetersPerSecond);
        }
        var can = RobotController.getCANStatus();
        telemetry.sample(new frc.robot.practice.MatchTelemetry.Sample(now - matchStart, matchTime, phaseName,
            RobotController.getBatteryVoltage(), RobotController.isBrownedOut(), amps, speed, aim.isActive(),
            SmartDashboard.getBoolean("Ready To Fire", false), rtHeld.getAsBoolean(), aim.isFiring(),
            SmartDashboard.getString("Shot Ready", ""), frc.robot.commands.VacuumCommand.vacuuming(),
            frc.robot.commands.VacuumCommand.dumping(), DefenseMode.active(), aim.poseUsable(),
            SmartDashboard.getNumber("Seconds Since Vision Fix", 999), ElevatorSubsystem.isRaised(),
            ElevatorSubsystem.isHanging(), SmartDashboard.getString("Power Budget", "").startsWith("SAG"),
            can.busOffCount, can.percentBusUtilization));
    }

    private void startMatch(String kind) {
        this.kind = kind;
        matchStart = Timer.getFPGATimestamp();
        lastFinished = shooter.finishedShots();
        notes.clear();
        var meta = new java.util.LinkedHashMap<String, Object>();
        meta.put("kind", kind);
        meta.put("robotMatch", matchNumber + 1);
        meta.put("startedAt", LocalDateTime.now().format(DateTimeFormatter.ISO_LOCAL_DATE_TIME));
        meta.put("fms", DriverStation.isFMSAttached());
        meta.put("event", DriverStation.getEventName());
        meta.put("fmsMatchType", DriverStation.getMatchType().toString());
        meta.put("fmsMatchNumber", DriverStation.getMatchNumber());
        meta.put("fmsReplay", DriverStation.getReplayNumber());
        meta.put("alliance", red() ? "red" : "blue");
        meta.put("station", DriverStation.getLocation().orElse(0));
        meta.put("autoName", autoName.get());
        meta.put("codeVersion", frc.robot.Robot.CODE_VERSION);
        for (String k : kCrew) {
            String key = "Match " + k;
            String v = SmartDashboard.getString(key, "");
            if (!v.equals(Preferences.getString(key, ""))) Preferences.setString(key, v);
            meta.put(k.equals("Battery") ? "batteryLabel" : k.toLowerCase(Locale.US), v);
        }
        telemetry = new frc.robot.practice.MatchTelemetry(meta);
        matchNumber++;
        inMatch = true;
        autoBalls = openBalls = closedBalls = passBalls = 0;
        lastShots = shooter.shotsStarted();
        teleopStart = liftTime = hangTime = -1;
        randomWinner = "";
        System.out.println("PRACTICE | match " + matchNumber + " started (" + kind + ")");
    }

    /** No FMS and no game data: pick who won auto at random, like a real match would decide it. */
    private void pickRandomWinner(boolean fms) {
        boolean on = SmartDashboard.getBoolean(kRandomKey, true);
        if (on != Preferences.getBoolean(kRandomKey, true)) Preferences.setBoolean(kRandomKey, on);
        String g = DriverStation.getGameSpecificMessage();
        if (fms || !on || (g != null && !g.isEmpty())) return;
        boolean weWon = random.nextBoolean();
        HubShiftTracker.setAutoWinnerOverride(weWon ? HubShiftTracker.AutoWinner.WE_WON : HubShiftTracker.AutoWinner.THEY_WON);
        randomized = true;
        boolean redWon = weWon == red();
        randomWinner = redWon ? "R" : "B";
        String msg = (weWon ? "WE won auto: our hub is CLOSED in shifts 1 and 3"
            : "THEY won auto: our hub is OPEN in shifts 1 and 3");
        SmartDashboard.putString("Practice Match", "Match " + matchNumber + ": " + msg);
        System.out.println("PRACTICE | random auto winner: " + msg);
    }

    private void finishMatch() {
        inMatch = false;
        lastResult = String.format(Locale.US, "Match %d: auto %d, open %d, closed %d, passes %d, climb %s",
            matchNumber, autoBalls, openBalls, closedBalls, passBalls, hangTime < 0 ? "none" : String.format(Locale.US, "hung at %.0f s", hangTime));
        SmartDashboard.putString("Practice Match", lastResult);
        System.out.println("PRACTICE | " + lastResult);
        if (telemetry == null) return;
        var summary = telemetry.summary();
        telemetry = null;
        double secs = ((Number) summary.get("durationSeconds")).doubleValue();
        if ("teleop_session".equals(kind) && secs < 20) return; // a quick enable: not worth a file
        String g = HubShiftTracker.gameData();
        summary.put("kind", kind);
        summary.put("autoWinner", g == null || g.isEmpty() ? null : Character.toUpperCase(g.charAt(0)) == 'R' ? "red" : "blue");
        summary.put("autoWinnerRandom", randomized);
        summary.put("batteryEndAtRest", RobotController.getBatteryVoltage());
        summary.put("logDir", logDir());
        String stamp = LocalDateTime.now().format(DateTimeFormatter.ofPattern("yyyyMMdd-HHmmss"));
        String name = stamp + "_" + kind + "_" + matchNumber + ".json";
        writer.execute(() -> saveMatch(name, summary));
    }

    /** Where DogLog's .wpilog files go: the USB stick when one is in, else the roboRIO. */
    private static String logDir() {
        return frc.robot.Telemetry.usbLogDrive() ? (new File("/u").isDirectory() ? "/u/logs" : "/U/logs") : "/home/lvuser/logs";
    }

    private void saveMatch(String name, java.util.Map<String, Object> summary) {
        try {
            var mapper = new com.fasterxml.jackson.databind.ObjectMapper();
            File dirF = new File(kMatchDir);
            dirF.mkdirs();
            File f = new File(dirF, name);
            mapper.writerWithDefaultPrettyPrinter().writeValue(f, summary);
            // index.json: every match file, newest last (the laptop sync reads it)
            String[] files = dirF.list((d, n) -> n.endsWith(".json") && !n.equals("index.json"));
            java.util.Arrays.sort(files);
            mapper.writeValue(new File(dirF, "index.json"), files);
            // one flat row per match for spreadsheets
            File csv = new File("/home/lvuser/shotmap", "matches.csv");
            boolean header = !csv.exists() || csv.length() == 0;
            @SuppressWarnings("unchecked") var sc = (java.util.Map<String, Object>) summary.get("scoring");
            @SuppressWarnings("unchecked") var b = (java.util.Map<String, Object>) summary.get("battery");
            @SuppressWarnings("unchecked") var cl = (java.util.Map<String, Object>) summary.get("climb");
            try (FileWriter w = new FileWriter(csv, true)) {
                if (header) w.write("file,kind,started,driver,operator,coach,battery,auto,auto_winner,auto_balls,balls_open,"
                    + "balls_closed,passes,ready_pct_rt,batt_start,batt_min,batt_end,brownouts,hang_s,duration_s\n");
                w.write(String.join(",", name, String.valueOf(summary.get("kind")), String.valueOf(summary.get("startedAt")),
                    csvCell(summary.get("driver")), csvCell(summary.get("operator")), csvCell(summary.get("coach")),
                    csvCell(summary.get("batteryLabel")), csvCell(summary.get("autoName")), String.valueOf(summary.get("autoWinner")),
                    String.valueOf(sc.get("autoBalls")), String.valueOf(sc.get("ballsHubOpen")), String.valueOf(sc.get("ballsHubClosed")),
                    String.valueOf(sc.get("passes")), String.valueOf(sc.get("readyWhileRtHeldPct")), String.valueOf(b.get("start")),
                    String.valueOf(b.get("min")), String.valueOf(b.get("end")), String.valueOf(b.get("brownouts")),
                    String.valueOf(cl.get("hangAtSeconds")), String.valueOf(summary.get("durationSeconds"))) + "\n");
            }
            // logs-index.json: the .wpilog files the laptop can download (port 5807)
            File logs = new File(logDir());
            String[] wl = logs.list((d, n) -> n.endsWith(".wpilog"));
            if (wl != null) {
                java.util.Arrays.sort(wl);
                var list = new java.util.ArrayList<java.util.Map<String, Object>>();
                for (String n : wl) list.add(java.util.Map.of("name", n, "bytes", new File(logs, n).length()));
                mapper.writeValue(new File("/home/lvuser/shotmap", "logs-index.json"), list);
            }
            System.out.println("PRACTICE | saved " + kMatchDir + "/" + name);
        } catch (Exception e) {
            System.out.println("PRACTICE | could not save the match: " + e.getMessage());
        }
    }

    private static String csvCell(Object o) {
        String v = o == null ? "" : o.toString();
        return v.contains(",") || v.contains("\"") ? "\"" + v.replace("\"", "\"\"") + "\"" : v;
    }

    private void writeStatus(boolean auto, boolean teleop, double matchTime, boolean fms) {
        MatchPhase.Phase p = MatchPhase.phase(auto, teleop, matchTime);
        String g = HubShiftTracker.gameData();
        Boolean redWon = (g == null || g.isEmpty()) ? null
            : Character.toUpperCase(g.charAt(0)) == 'R' ? Boolean.TRUE
            : Character.toUpperCase(g.charAt(0)) == 'B' ? Boolean.FALSE : null;
        boolean redOn = MatchPhase.hubActive(auto, teleop, matchTime, true, redWon);
        boolean blueOn = MatchPhase.hubActive(auto, teleop, matchTime, false, redWon);
        String json = String.format(Locale.US,
            "{\"t\":%.2f,\"mode\":\"%s\",\"fms\":%b,\"matchTime\":%.2f,\"phase\":\"%s\",\"phaseLeft\":%.2f,"
            + "\"nextChange\":%.2f,\"alliance\":\"%s\",\"autoWinner\":\"%s\",\"randomWinner\":%b,"
            + "\"redHub\":%b,\"blueHub\":%b,\"match\":%d,\"inMatch\":%b,\"autoBalls\":%d,\"openBalls\":%d,"
            + "\"closedBalls\":%d,\"passBalls\":%d,\"lift\":%.1f,\"hang\":%.1f,\"climbing\":%b,\"hanging\":%b,"
            + "\"ready\":%b,\"aimOn\":%b,\"poseTrusted\":%b,\"battery\":%.2f,\"last\":\"%s\"}",
            Timer.getFPGATimestamp(), auto ? "auto" : teleop ? "teleop" : "disabled", fms, matchTime, p.name(),
            p.remaining(), MatchPhase.secondsToNextChange(teleop, matchTime), red() ? "red" : "blue",
            redWon == null ? "" : redWon ? "red" : "blue", randomized, redOn, blueOn, matchNumber, inMatch,
            autoBalls, openBalls, closedBalls, passBalls, liftTime, hangTime, ElevatorSubsystem.isRaised(),
            ElevatorSubsystem.isHanging(), SmartDashboard.getBoolean("Ready To Fire", false), aim.isActive(),
            aim.poseUsable(), RobotController.getBatteryVoltage(), lastResult.replace("\"", "'"));
        String d = dir;
        writer.execute(() -> {
            try {
                File tmp = new File(d, "match.json.tmp");
                Files.writeString(tmp.toPath(), json);
                Files.move(tmp.toPath(), new File(d, "match.json").toPath(), StandardCopyOption.REPLACE_EXISTING,
                    StandardCopyOption.ATOMIC_MOVE);
            } catch (Exception e) {
                // the screen just shows "no robot" until the next write works
            }
        });
    }
}
