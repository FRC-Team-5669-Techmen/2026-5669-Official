package frc.robot.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

/**
 * Wrapper for the Logitech G X56 H.O.T.A.S. flight controller.
 *
 * <p>The X56 shows up as TWO separate USB devices: the stick and the throttle.
 * Assign them to {@link Constants.Hotas#kStickPort} and
 * {@link Constants.Hotas#kThrottlePort} in the Driver Station USB Order tab.
 *
 * <p>All axis values returned here are already deadbanded and sign-corrected for
 * WPILib's field convention (forward = +X, left = +Y, counter-clockwise = +rotation),
 * so callers can feed them straight into a swerve request the same way they would
 * the negated Xbox axes.
 *
 * <p>Axis and button indices live in {@link Constants.Hotas} — verify them against
 * the Driver Station USB tab (see docs/X56-HOTAS-SETUP.md) since HID ordering can
 * differ between firmware versions.
 */
public class HotasX56 {
    private final CommandJoystick stick;
    private final CommandJoystick throttle;

    public HotasX56(int stickPort, int throttlePort) {
        stick = new CommandJoystick(stickPort);
        throttle = new CommandJoystick(throttlePort);
    }

    /** True when the flight stick is plugged in and assigned to its port. */
    public boolean isConnected() {
        return DriverStation.isJoystickConnected(stick.getHID().getPort());
    }

    /** True when the separate throttle unit is plugged in. */
    public boolean isThrottleConnected() {
        return DriverStation.isJoystickConnected(throttle.getHID().getPort());
    }

    // ==========================================
    // Drive axes
    // ==========================================

    /** Stick pitch: push forward to drive downfield. Range [-1, 1], deadbanded. */
    public double getForward() {
        // HID reports stick pushed forward as negative Y, same as an Xbox stick
        double raw = -stick.getRawAxis(Constants.Hotas.kStickPitchAxis);
        return MathUtil.applyDeadband(raw, Constants.Hotas.kStickDeadband);
    }

    /** Stick roll: tilt left to strafe left. Range [-1, 1], deadbanded. */
    public double getStrafe() {
        // Positive HID X is right; WPILib wants left-positive
        double raw = -stick.getRawAxis(Constants.Hotas.kStickRollAxis);
        return MathUtil.applyDeadband(raw, Constants.Hotas.kStickDeadband);
    }

    /** Stick twist: twist left (CCW) to rotate CCW. Range [-1, 1], deadbanded wide. */
    public double getTwist() {
        double raw = -stick.getRawAxis(Constants.Hotas.kStickTwistAxis);
        return MathUtil.applyDeadband(raw, Constants.Hotas.kTwistDeadband);
    }

    /**
     * Right throttle lever mapped to a speed governor in
     * [{@link Constants.Hotas#kMinThrottleScale}, 1.0].
     * Full forward = 100% speed, pulled all the way back = the minimum crawl speed.
     * Falls back to full speed if the throttle unit is unplugged.
     */
    public double getThrottleScale() {
        if (!isThrottleConnected()) {
            return 1.0;
        }
        double raw = throttle.getRawAxis(Constants.Hotas.kRightThrottleAxis);
        if (Constants.Hotas.kThrottleInverted) {
            raw = -raw;
        }
        double scale = (raw + 1.0) / 2.0; // [-1, 1] -> [0, 1]
        return MathUtil.clamp(scale, Constants.Hotas.kMinThrottleScale, 1.0);
    }

    // ==========================================
    // Stick buttons
    // ==========================================

    /** Index-finger trigger. */
    public Trigger trigger() {
        return stick.button(Constants.Hotas.kTriggerButton);
    }

    /** Red thumb button on top of the stick. */
    public Trigger buttonA() {
        return stick.button(Constants.Hotas.kButtonA);
    }

    public Trigger buttonB() {
        return stick.button(Constants.Hotas.kButtonB);
    }

    public Trigger buttonC() {
        return stick.button(Constants.Hotas.kButtonC);
    }

    public Trigger buttonD() {
        return stick.button(Constants.Hotas.kButtonD);
    }

    /** Pinkie lever on the front of the grip. */
    public Trigger pinkie() {
        return stick.button(Constants.Hotas.kPinkieButton);
    }

    /** Main 8-way hat, up direction. */
    public Trigger povUp() {
        return stick.povUp();
    }

    /** Main 8-way hat, down direction. */
    public Trigger povDown() {
        return stick.povDown();
    }

    /** Raw access for extra bindings (mini stick, second hat, etc.). */
    public CommandJoystick getStick() {
        return stick;
    }

    /** Raw access to the throttle unit (SW1-6, TGL1-4, rotaries, etc.). */
    public CommandJoystick getThrottle() {
        return throttle;
    }
}
