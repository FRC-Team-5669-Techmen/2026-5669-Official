package frc.robot.aiming;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;

class TargetPickerTest {
    // The real field numbers from Constants.Aim (2026 welded field).
    static TargetPicker picker() {
        return new TargetPicker(new TargetPicker.Field(16.541, 8.069, 4.6255, 4.0346, 0.597,
            4.19, 0.25, 0.94, 1.65, 1.40, 1.00, 0.30));
    }

    static Translation2d p(double x, double y) { return new Translation2d(x, y); }

    @Test
    void scoresInsideTheAllianceZonePassesFromTheNeutralZone() {
        TargetPicker t = picker();
        assertEquals(TargetPicker.Kind.SCORE, t.pick(p(3.0, 2.0), p(3.0, 2.0), false, TargetPicker.Mode.AUTO).kind());
        assertEquals(TargetPicker.Kind.PASS, t.pick(p(8.0, 2.0), p(8.0, 2.0), false, TargetPicker.Mode.AUTO).kind());
    }

    @Test
    void neverAutoScoresPastTheLegalLine() {
        // G407: robot center 4.30 m from the blue wall is past the 4.19 m safe line.
        TargetPicker t = picker();
        assertEquals(TargetPicker.Kind.PASS, t.pick(p(4.30, 1.0), p(4.30, 1.0), false, TargetPicker.Mode.AUTO).kind());
        assertFalse(t.legalToScore(p(4.30, 1.0), false));
        assertTrue(t.legalToScore(p(4.10, 1.0), false));
    }

    @Test
    void hysteresisKeepsItFromFlickering() {
        TargetPicker t = picker();
        t.pick(p(5.0, 1.0), p(5.0, 1.0), false, TargetPicker.Mode.AUTO);              // PASS
        assertEquals(TargetPicker.Kind.PASS, t.pick(p(4.10, 1.0), p(4.10, 1.0), false, TargetPicker.Mode.AUTO).kind());
        assertEquals(TargetPicker.Kind.SCORE, t.pick(p(3.90, 1.0), p(3.90, 1.0), false, TargetPicker.Mode.AUTO).kind());
    }

    @Test
    void passGoesToTheCornerOnTheRobotsSide() {
        TargetPicker t = picker();
        Translation2d low = t.pick(p(8.0, 1.0), p(8.0, 1.0), false, TargetPicker.Mode.PASS).point();
        assertEquals(0.94, low.getX(), 1e-9);
        assertEquals(1.65, low.getY(), 1e-9);
        Translation2d high = t.pick(p(8.0, 7.0), p(8.0, 7.0), false, TargetPicker.Mode.PASS).point();
        assertEquals(8.069 - 1.65, high.getY(), 1e-9);
    }

    @Test
    void redMirrorsAcrossTheField() {
        TargetPicker t = picker();
        Translation2d pt = t.pick(p(8.0, 1.0), p(8.0, 1.0), true, TargetPicker.Mode.PASS).point();
        assertEquals(16.541 - 0.94, pt.getX(), 1e-9);
        assertEquals(TargetPicker.Kind.SCORE, t.pick(p(13.0, 1.0), p(13.0, 1.0), true, TargetPicker.Mode.AUTO).kind());
    }

    @Test
    void hubInTheLaneUsesTheBackup() {
        // From just behind our hub, off its centerline (x 6.5, y 3.6): the corner lane
        // clips the padded hub, the backup lane (closer to the side wall) is clear.
        TargetPicker t = picker();
        Translation2d launcher = p(6.5, 3.6);
        assertTrue(t.blocked(launcher, p(0.94, 1.65), false));
        TargetPicker.Target tg = t.pick(launcher, launcher, false, TargetPicker.Mode.PASS);
        assertTrue(tg.laneClear());
        assertTrue(tg.name().contains("backup"), tg.name());
        assertEquals(1.0, tg.point().getY(), 1e-9);
    }

    @Test
    void straightBehindOurHubEveryLaneIsBlocked() {
        // Right behind our hub on its centerline: every lane crosses it. Hold the pass.
        TargetPicker t = picker();
        Translation2d launcher = p(5.6, 4.03);
        TargetPicker.Target tg = t.pick(launcher, launcher, false, TargetPicker.Mode.PASS);
        assertFalse(tg.laneClear(), tg.name());
    }

    @Test
    void clearLaneUsesTheCorner() {
        TargetPicker t = picker();
        assertFalse(t.blocked(p(8.0, 1.0), p(0.94, 1.65), false));
    }

    // With the shuttle settings (12 m max pass, relay 6.0 m from our wall, 1.65 m from the side).
    static TargetPicker shuttlePicker() {
        return new TargetPicker(new TargetPicker.Field(16.541, 8.069, 4.6255, 4.0346, 0.597,
            4.19, 0.25, 0.94, 1.65, 1.40, 1.00, 0.30, 12.0, 6.0, 1.65));
    }

    @Test
    void deepInTheirZoneItShuttlesToTheNeutralZone() {
        TargetPicker t = shuttlePicker();
        // Blue robot at x 15.0 (in red's alliance zone), low side: our corner is 14 m away.
        TargetPicker.Target tg = t.pick(p(15.0, 1.2), p(15.0, 1.2), p(15.0, 1.2), false, TargetPicker.Mode.AUTO, 12.0);
        assertEquals(TargetPicker.Kind.PASS, tg.kind());
        assertTrue(TargetPicker.isRelay(tg), tg.name());
        assertEquals(6.0, tg.point().getX(), 1e-9);
        assertEquals(1.65, tg.point().getY(), 1e-9);
        assertTrue(tg.laneClear());
        assertTrue(t.inTheirZone(p(15.0, 1.2), false, 4.029));
        assertFalse(t.inTheirZone(p(10.0, 1.2), false, 4.029));
    }

    @Test
    void inRangeItStillPassesStraightHome() {
        TargetPicker t = shuttlePicker();
        TargetPicker.Target tg = t.pick(p(11.0, 1.2), p(11.0, 1.2), p(11.0, 1.2), false, TargetPicker.Mode.AUTO, 12.0);
        assertFalse(TargetPicker.isRelay(tg));
        assertEquals(0.94, tg.point().getX(), 1e-9);
    }

    @Test
    void redShuttleMirrors() {
        TargetPicker t = shuttlePicker();
        TargetPicker.Target tg = t.pick(p(1.5, 7.0), p(1.5, 7.0), p(1.5, 7.0), true, TargetPicker.Mode.AUTO, 12.0);
        assertTrue(TargetPicker.isRelay(tg));
        assertEquals(16.541 - 6.0, tg.point().getX(), 1e-9);
        assertEquals(8.069 - 1.65, tg.point().getY(), 1e-9);
    }

    @Test
    void behindTheirHubEveryShuttleLaneIsBlocked() {
        // Right behind the red hub (x 11.9) at hub height: every line home crosses it.
        TargetPicker t = shuttlePicker();
        TargetPicker.Target tg = t.pick(p(14.0, 4.0), p(14.0, 4.0), p(14.0, 4.0), false, TargetPicker.Mode.AUTO, 12.0);
        assertFalse(tg.laneClear(), tg.name());
    }

    @Test
    void stopsBeforeAHubSlidesIntoTheLane() {
        // Driving across the field toward the hub's shadow: clear from here, blocked from where the
        // robot will be when the last ball leaves = hold now, before the hub is in the way.
        TargetPicker t = shuttlePicker();
        Translation2d now = p(7.5, 3.3), later = p(7.5, 4.0);
        Translation2d corner = p(0.94, 1.65);
        assertFalse(t.blocked(now, corner, false));
        assertTrue(t.blocked(later, corner, false));
        assertFalse(t.laneClear(now, later, corner, false));
    }
}
