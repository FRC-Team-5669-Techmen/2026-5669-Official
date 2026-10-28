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
        boolean auto = DriverStation.isAutonomousEnabled();
        boolean teleop = DriverStation.isTeleopEnabled();
        double now = Timer.getFPGATimestamp();
        double matchTime = DriverStation.getMatchTime();
        boolean fms = DriverStation.isFMSAttached();

        // ---- match start / end ----
        if (auto && !wasAuto) startMatch();
        if (teleop && !wasTeleop) {
            if (!inMatch) startMatch();       // teleop-only practice still gets a scorecard
            teleopStart = now;
            pickRandomWinner(fms);
        }
        if (!auto && !teleop && (wasAuto || wasTeleop)) {
            if (wasTeleop && inMatch) finishMatch(); // teleop ended (or disabled): write the row
            if (randomized) {                       // back to the dashboard's choice
                HubShiftTracker.setAutoWinnerOverride(chooserPick.get());
                randomized = false;
            }
        }
        wasAuto = auto;
        wasTeleop = teleop;

        // ---- scorecard ----
        int shots = shooter.shotsStarted();
        if (shots < lastShots) lastShots = shots; // stats were reset
        while (lastShots < shots) {
            lastShots++;
            if (!inMatch) continue;
            if (auto) autoBalls++;
            else if (aim.passing()) passBalls++;
            else if (HubShiftTracker.ourHubStateAt(kFlightGuess) == HubShiftTracker.HubState.CLOSED) closedBalls++;
            else openBalls++;
        }
        if (inMatch && teleop) {
            if (liftTime < 0 && ElevatorSubsystem.isRaised()) liftTime = now - teleopStart;
            if (hangTime < 0 && ElevatorSubsystem.isHanging()) hangTime = now - teleopStart;
        }

        // ---- status for the match screen (10 times a second) ----
        if (++loops % 5 == 0 && dir != null) writeStatus(auto, teleop, matchTime, fms);
    }

    private void startMatch() {
        matchNumber++;
        inMatch = true;
        autoBalls = openBalls = closedBalls = passBalls = 0;
        lastShots = shooter.shotsStarted();
        teleopStart = liftTime = hangTime = -1;
        randomWinner = "";
        System.out.println("PRACTICE | match " + matchNumber + " started");
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
        String row = String.format(Locale.US, "%s,%d,%s,%d,%d,%d,%d,%.1f,%.1f,%.2f%n",
            LocalDateTime.now().format(DateTimeFormatter.ISO_LOCAL_DATE_TIME), matchNumber,
            randomWinner.isEmpty() ? "?" : randomWinner, autoBalls, openBalls, closedBalls, passBalls,
            liftTime, hangTime, RobotController.getBatteryVoltage());
        lastResult = String.format(Locale.US, "Match %d: auto %d, open %d, closed %d, passes %d, climb %s",
            matchNumber, autoBalls, openBalls, closedBalls, passBalls, hangTime < 0 ? "none" : String.format(Locale.US, "hung at %.0f s", hangTime));
        SmartDashboard.putString("Practice Match", lastResult);
        System.out.println("PRACTICE | " + lastResult);
        writer.execute(() -> {
            File f = new File("/home/lvuser/shotmap", "practice-matches.csv");
            boolean header = !f.exists() || f.length() == 0;
            if (f.length() > 1_000_000) return;
            try (FileWriter w = new FileWriter(f, true)) {
                if (header) w.write("time,match,auto_winner,auto_balls,balls_hub_open,balls_hub_closed,passes,"
                    + "climb_lift_s_into_teleop,hang_s_into_teleop,battery_v\n");
                w.write(row);
            } catch (Exception e) {
                System.out.println("PRACTICE | could not save the match: " + e.getMessage());
            }
        });
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
