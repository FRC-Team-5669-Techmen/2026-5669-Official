package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.AngularVelocity;

public final class Constants {

    public static final class Operator {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final double kDeadband = 0.05;
        public static final double kRotationalDeadband = 0.05;
        public static final AngularVelocity kMaxAngularRate = RotationsPerSecond.of(0.75);
    }

    /**
     * Logitech G X56 H.O.T.A.S. (stick + throttle = two separate USB devices).
     * The stick is an optional alternative to the driver Xbox controller; when it is
     * unplugged every HOTAS binding is inert and the Xbox controller works as before.
     *
     * Axis/button indices vary between firmware/OS versions — verify each one in the
     * Driver Station USB tab before competition. See docs/X56-HOTAS-SETUP.md.
     */
    public static final class Hotas {
        // SWITCHED OFF: only the two Xbox controllers drive. Set true to use the
        // X56 for practice. Even when on, it can never use ports 0 or 1.
        public static final boolean kEnabled = false;

        // USB ports (drag the devices to these slots in the DS USB Order tab)
        public static final int kStickPort = 2;
        public static final int kThrottlePort = 3;

        // --- Stick axes ---
        public static final int kStickRollAxis = 0;   // X: left/right -> strafe
        public static final int kStickPitchAxis = 1;  // Y: forward/back -> translation
        public static final int kStickTwistAxis = 2;  // Rz twist -> rotation

        // --- Stick buttons (1-indexed, WPILib convention) ---
        public static final int kTriggerButton = 1;   // index-finger trigger
        public static final int kButtonA = 2;         // red thumb button
        public static final int kButtonB = 3;
        public static final int kButtonC = 4;
        public static final int kButtonD = 5;
        public static final int kPinkieButton = 6;    // pinkie lever

        // --- Throttle axes ---
        public static final int kRightThrottleAxis = 1;
        // Most HID stacks report a throttle pushed fully forward as -1.0
        public static final boolean kThrottleInverted = true;

        // --- Tuning ---
        public static final double kStickDeadband = 0.08;
        public static final double kTwistDeadband = 0.15;  // twist is noisy; needs a wider deadband
        // Throttle acts as an analog speed governor; never let it scale the robot to a dead stop
        public static final double kMinThrottleScale = 0.15;
    }

    public static final class Shooter {
        public static final int kLeaderId = 16;
        public static final int kFollowerId = 17;
        public static final boolean kLeaderInverted = true;
        public static final boolean kFollowerInverted = false;
        // NOTE: requests above kMaxRPM are clamped in ShooterSubsystem.runAtRPM()
        // Operator LT spin-up. 5400 is about 93% of the Kraken's free speed (~5800 RPM with FOC),
        // so the speed loop still has headroom to hold it steady as the battery drains.
        public static final double kFastTargetRPM = 5400.0;
        public static final double kSlowTargetRPM = 2500.0;
        public static final double kReverseRPM = -1000.0;
        public static final double kTestingRPM = 1500.0;
        // Hard ceiling = the motor's own top speed (Kraken X60 with FOC). Was 3100 until Oct 25,
        // which could not shoot high enough to reach the target.
        public static final double kMaxRPM = 5800.0;
        // Driver D-pad Left idle speed. 2000 RPM = about 33.3 rotations per second
        // (the shooter code divides by 60 for you).
        public static final double kIdleRPM = 2000.0;
        public static final double kDecelerateStep = 60.0;
        public static final double kIdleAccelerateStep = 50.0;
        public static final double kP = 0.11;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kV = 0.12;
        // Per motor (2 motors). 60 A supply each keeps a cold spin-up from browning out the battery.
        public static final double kSupplyCurrentLimit = 60.0;
        public static final double kStatorCurrentLimit = 120.0;
        // Spin-up ramp: 0 to full voltage in 0.25 s, easy on the belt, still at speed in about a second.
        public static final double kVoltageRampPeriod = 0.25;
        public static final double kDutyCycleRampPeriod = 0.0;
    }

    public static final class Index {
        public static final int kMotorId = 13; // Tuner X name: "Spindex"
        public static final boolean kInverted = false;
        public static final double kForwardSpeed = 12.0; // volts
        public static final double kReverseSpeed = -6.0; // volts
    }

    public static final class ShooterIntake {
        public static final int kMotorId = 15; // Tuner X name: "UpTakeChannelForShooter"
        public static final boolean kInverted = true;
        // Duty cycle, -1 to 1. Was 2.0, which is out of range; 1.0 = full power, stated plainly.
        public static final double kForwardSpeed = 1.0;
        public static final double kReverseSpeed = -0.6;
    }

    /**
     * Turret ("Goober"), Talon FX 14. Turns the shooter, and the Limelight on it, about 270°.
     *
     * <p>HOME = one END of the travel, lined up with the spot mark. From HOME it can only turn
     * RIGHT (clockwise seen from above) about 270°, then back. It can't go all the way around
     * because the motor and Limelight wires run up to it. Nothing else stops it, so the soft
     * limits below are what protect the wires.
     *
     * <p>ALWAYS power on with the turret on the HOME mark: the motor calls wherever it is at
     * power-on "0". Deploying code does NOT re-zero it.
     */
    public static final class Turret {
        public static final int kMotorId = 14; // Tuner X name: "AngleShooter"

        // In the code, POSITIVE = turning RIGHT = away from HOME. Test: at HOME, TAP Operator
        // D-pad Right. It must turn RIGHT (away from home, like the photos). If it turns LEFT
        // (past home, into the wires), stop and flip this.
        public static final boolean kMotorInverted = false;

        // ---- Range, in motor rotations from HOME (dashboard "Turret Position (rot)") ----
        // HOME is 0. The far end (270° from HOME) is set from the DASHBOARD: power on at HOME,
        // hold Operator D-pad Right until the turret is just SHORT of where the wires get
        // tight, press "Set Turret Far End (turret at far end)". It's saved on the roboRIO
        // and shows as "Turret Far End (rot)".
        // kEndPosition is only a backup, used when nothing is saved: type the
        // "Turret Far End (rot)" number here too. 0.0 = none: until the far end is set,
        // auto-aim does nothing and NOTHING stops the turret past 270°, so watch the wires.
        // Oct 24: the far end first read 32.49; the team then decided on 36.
        public static final double kEndPosition = 36;
        // "Set Turret Far End" refuses to save anything closer to HOME than this (rotations),
        // so pressing it by mistake at HOME can't lock the turret in place.
        public static final double kMinEndPosition = 1.0;
        // How far kEndPosition is from HOME, in degrees. Turns rotations into degrees.
        // Oct 25 (programming lead): the far end is a little past 270, about 280 degrees from HOME.
        public static final double kEndDegrees = 280.0;
        // Auto-aim and the search sweep stay this far inside each end, so they never push on
        // the limits. The D-pad jog can go all the way to the ends.
        public static final double kAimMarginDegrees = 10.0;

        // Strength cap. Low on purpose: limits how hard it can pull on the wires if it gets stuck.
        public static final double kStatorCurrentLimit = 30.0;  // amps

        public static final double kSpeedMultiplier = 1.0;
        // Operator D-pad Left/Right speed (percent power). Was 0.2; slower is easier to aim by hand.
        public static final double kManualJogSpeed = 0.12;
        // Slow scan speed used while hunting for a target with no Limelight lock
        public static final double kSearchSpeed = 0.15;

        public static final double kP = 0.0068;
        public static final double kI = 0.00;
        public static final double kD = 0.00;
        public static final double kToleranceDegrees = 1.0;
        public static final double kMaxOutput = 0.8;
    }

    public static final class Auton {
        public static final double kDriveSpeed = 0.5;
        public static final double kTimeoutSeconds = 5.0;
    }

    public static final class Sim {
        public static final double kLoopPeriod = 0.002;
    }

    /**
     * Hood ("Gooba"), Kraken X44 on Talon FX 18 (Tuner X name: "Shooter Top"). Tilts up and down
     * to change the shot. Same hood code as last season (it was on ID 51 then).
     *
     * <p>0 = power-on position, so ALWAYS power on with the hood all the way DOWN. Deploying
     * code does NOT re-zero it. Positive = hood UP.
     */
    public static final class Gooba {
        // false = the motor is never created and every hood button does nothing.
        public static final boolean kHoodEnabled = true;
        public static final int kMotorId = 18;
        // Test: tap Operator D-pad UP. The hood must go UP. If it goes down, flip this.
        public static final boolean kInverted = false;
        // ---- Soft limits, in motor rotations (dashboard "Hood Position (rot)") ----
        // NOT measured on this robot yet: 0.0 / 10.7 are last season's numbers. To measure:
        // power on with the hood all the way DOWN, click "Hood Limits OFF (measure)", jog
        // with Operator D-pad Up until the hood JUST touches the top, read "Hood Position (rot)",
        // then D-pad Down until it just touches the bottom and read it again. Put a hair inside
        // each number here, then click the switch again to turn the limits back on.
        // BOTTOM, measured Oct 25 (Elastic): pushed down by hand at power-on = 0 (the hard stop).
        // Let go, it springs back up to about 0.42-0.51; the motor jogging down got it to 0.24.
        // 0.2 = just above the hard stop, so the motor never grinds into it.
        public static final double kBottomPosition = 0.2;
        // TOP, measured Oct 25 (Elastic): pushed all the way up = 3.89 (last season's hood
        // went to 10.7, so the gearing is different now). 3.8 = a hair below the top stop.
        public static final double kTopPosition = 3.8;

        public static final double kP = 2.4;
        public static final double kI = 0.0;
        public static final double kD = 0.1;
        public static final double kCruiseVelocity = 80.0;
        public static final double kAcceleration = 160.0;
        public static final double kSupplyCurrentLimit = 40.0;
        public static final double kStatorCurrentLimit = 30.0;  // amps, limits the push if it jams

        // Operator Y presets (rotations)
        public static final double kPositionStowed = 0.2;  // = kBottomPosition
        // Last season's 3.5 out of 10.7, scaled to this hood's 3.89 travel (same share of the way up)
        public static final double kPositionDeployed = 1.3;
        // Operator D-pad Up/Down jog: rotations per 20 ms loop (0.04 = 2 rotations per second)
        public static final double kManualStep = 0.04;
    }

    /**
     * Ground intake pivot: two motors that raise and lower the intake together.
     * Motor 20 is the leader (the one the code talks to); motor 21 copies it.
     */
    public static final class GroundIntakePivot {
        public static final int kLeaderId = 20;   // Tuner X name: "Right Ground Intake Raise"
        public static final int kFollowerId = 21; // Tuner X name: "Left Ground Intake Raise"

        // In the code, POSITIVE always means UP. If the intake moves the wrong way,
        // flip this (it reverses both motors). Never "fix" direction by changing signs.
        public static final boolean kLeaderInverted = true;  // flipped: both motors ran backwards in testing

        // The two motors are mounted mirrored, so 21 spins opposite to 20.
        // Motors fight (lots of current, no movement)? Change to Aligned.
        public static final MotorAlignmentValue kFollowerAlignment = MotorAlignmentValue.Opposed;

        // ---- Positions, in motor rotations ----
        // The motors call wherever the intake is at POWER-ON "UP = 0". So before every
        // power-on, push the intake ALL the way up until it touches the top. (Or, with the
        // robot on, push it up and press "Zero Intake" in Elastic.) If it sagged even a
        // little, every position is off by that much: Oct 25 it was 0.9 low, so DOWN hit
        // the floor at full speed and UP stopped too low for the brake to hold it.
        public static final double kUpPosition = 0.0;
        // DOWN is below UP, so it's a NEGATIVE number. To measure it: jog down with the
        // Operator D-pad until the intake JUST touches the ground or hard stop, read
        // "Intake Position (rot)" on the dashboard, and type that number here.
        // 0.0 means "not set yet": while it's 0.0, the Driver A button does nothing.
        // MEASURED on the robot (Oct 23, Elastic): resting on the floor, the right side
        // (motor 20) read -5.73 and the left -5.83. -5.7 = just above the floor.
        // More negative = further down. The DOWN soft limit follows this number.
        public static final double kDownPosition = -5.7;
        // The A button stops this many rotations SHORT of kDownPosition, so the motors
        // aren't stalled pushing into the ground.
        public static final double kDownBackoff = 0.1;
        // The DOWN soft limit sits this far past kDownPosition. (No UP soft limit.)
        public static final double kSoftLimitMargin = 0.25;

        // ---- Motion Magic speed: how fast the A button moves it (see the speed-up table) ----
        public static final double kCruiseVelocity = 25.0;  // top speed, rotations per second
        public static final double kAcceleration = 130.0;   // how fast it reaches top speed (rot/s per second)
        public static final double kJerk = 1300.0;          // how gently it starts/stops accelerating (bigger = sharper)

        // ---- Holding: how the motors keep it on target ----
        public static final double kP = 3.0;   // push-back strength per rotation of error. Sags = raise, bounces = lower
        public static final double kD = 0.1;   // damping: calms bouncing. Raise a little if it overshoots
        public static final double kV = 0.12;  // volts per rot/s, the Kraken's built-in speed constant. Leave it

        // Operator D-pad jog speed (volts). The slow speed from the last test.
        public static final double kJogVolts = 1.0;

        // ---- UP: how far below the top the A button stops ----
        // The A button sends it UP to this many rotations BELOW the top hard stop, so it
        // never touches the top. Bigger = stops lower. Smaller = closer to the top.
        // It keeps HOLDING it there (a faint hum is normal), so it can't drop mid-match.
        public static final double kUpHoldOffset = 0.5;  // 0.5 = the right UP spot (Oct 24 test)

        // ---- ZERO WRONG warning ----
        // If DOWN stops more than this many rotations short of the floor, UP = 0 must be
        // wrong, and "Intake Status" says ZERO WRONG. (A ball under the intake can also
        // trigger it; the next clean DOWN clears it.)
        public static final double kZeroWrongTolerance = 0.3;

        // ---- Feeling an end ("stall" = pushing hard but not moving) ----
        // If the motors stall against something during DOWN, they stop pushing.
        // Stall = more than kStallAmps while slower than kStallSpeed, for kStallSeconds.
        public static final double kStallAmps = 15.0;
        public static final double kStallSpeed = 0.5;     // rot/s
        public static final double kStallSeconds = 0.2;
        public static final double kAtTargetTolerance = 0.3; // "close enough", in rotations
        public static final double kMoveTimeout = 2.0;       // seconds, safety cap on a move

        // Strength cap per motor. Limits how hard it can push if it jams or hits an end.
        public static final double kStatorCurrentLimit = 40.0;  // amps, each motor
    }

    /**
     * Climber (an elevator): rises, latches on, then pulls the robot up. Talon FX 19.
     * Hold Driver D-pad Up/Down to move it, release to stop.
     */
    public static final class Elevator {
        public static final int kMotorId = 19;  // Tuner X name: "Elevator"

        // In the code, positive = UP. If D-pad Up makes it go DOWN, flip this.
        public static final boolean kElevatorInverted = false;

        // Speed while a D-pad button is held (volts, battery is ~12 V). Starts gentle.
        // Raise in 0.5 V steps once the direction is confirmed.
        public static final double kElevatorVolts = 2.0;  // was 1.5 (first test was good)

        // Strength cap. Probably too low to lift the whole robot; see notes in the PR.
        public static final double kStatorCurrentLimit = 40.0;  // amps

        // ---- Soft limits: the motor stops by itself just before the top and the bottom ----
        // Even with the D-pad still held, it stops there, so it never stalls into the ends.
        // Positions are in motor rotations, from "Elevator Position (rot)" on the dashboard.
        // 0 = where it was at power-on, so ALWAYS power on with the elevator fully DOWN.
        // To measure:
        //   1. Hold D-pad Up until it's just BELOW the top (not touching). Type that number in kTopLimit.
        //   2. Hold D-pad Down until it's just ABOVE the bottom. Type that number in kBottomLimit
        //      (a small number, a bit above 0).
        // While kTopLimit is 0.0 the limits are OFF (not measured yet).
        public static final double kTopLimit = 125.1;
        public static final double kBottomLimit = 0.0;
    }

    /**
     * Ground intake ROLLER (the spinning wheels that pick up balls). The motor is
     * powered but NOT on CAN yet. While kIntakeRollerEnabled is false, the code never
     * creates the motor (so no CAN errors) and Driver Right Trigger does nothing.
     * When it's on CAN: set kMotorId to its ID in Tuner X, then set the flag to true.
     */
    public static final class IntakeRoller {
        public static final boolean kIntakeRollerEnabled = true;  // on CAN now (Oct 2026)
        public static final int kMotorId = 22;  // Tuner X name: "Intake motor(22)"
        public static final boolean kInverted = false;  // flip if it spins balls OUT instead of in
        public static final double kRollerVolts = 12.0; // spin speed while Driver RT is held (volts). Full power (Oct 25; was 9.0, before that 6.0)
        public static final double kReverseVolts = -9.0; // Driver LT (hold): spit balls back out
        public static final double kStatorCurrentLimit = 60.0;  // amps (was 40, raised with full power)
    }

    public static final class Limelight {
        // HUB AprilTags the turret may aim at: the tag in the MIDDLE of each of the 3 hub faces
        // we shoot at (the face toward the center of the field is left out). Checked against
        // the official WPILib 2026 field layout. Once the Driver Station says our alliance, the
        // Limelight only looks for OUR 3 (both hubs until then), so it never aims at theirs.
        public static final int[] kRedHubTagIds = {2, 5, 10};
        public static final int[] kBlueHubTagIds = {18, 21, 26};

        // Offseason remount: camera is horizontal and centered on the shooter,
        // so tx aims the turret directly and ty feeds the distance trig in
        // LimelightSubsystem.distanceToTarget().
        // TODO: re-measure the mount angle and lens height on the new mount.
        public static final double kMountAngleDegrees = 36.4;
        public static final double kLensHeightMeters = 0.0;
        public static final double kAprilTagHeightMeters = 1.12395;
    }

    /**
     * SHOOT FROM ANYWHERE (Oct 2026). The robot knows where it is on the field (wheel odometry
     * + gyro, corrected by the Limelight seeing ANY AprilTag), so it can point the turret, set
     * the hood and pick the shooter speed for our hub from any spot, with or without the hub
     * tags in view. See frc.robot.aiming and CLAUDE.md "Shoot from anywhere".
     *
     * <p>!!! The geometry below is NOT MEASURED YET. Until it is, aiming and vision position
     * are off by however wrong these are. Measure on the robot, then set kGeometryMeasured.
     * Directions: x = toward the robot's FRONT (the side the field-centric "forward" faces when
     * the gyro is reset), y = toward the robot's LEFT, angles counter-clockwise seen from above.
     */
    public static final class Aim {
        // Flip to true once every number marked MEASURE has been measured on the robot.
        // While false the dashboard shows "AIM GEOMETRY NOT MEASURED".
        // Oct 25: measured by the programming lead (camera height still an estimate).
        public static final boolean kGeometryMeasured = true;

        // ---- Turret ----
        // MEASURE: where the turret's pivot (the center it turns around) is, from the robot's
        // center, in meters. Forward = +x, left = +y.
        // MEASURED Oct 25: 6.475 in from the robot center, behind it and to its LEFT (photo:
        // "back left, not entirely left"). Split from the photo as ~5.3 in back, ~3.7 in left; a
        // 2 in error here moves the aim less than 1 degree at 4 m.
        public static final double kTurretPivotX = -5.3 * 0.0254;
        public static final double kTurretPivotY = 3.7 * 0.0254;
        // MEASURE: which way the shooter points when the turret is at HOME, in degrees from the
        // robot's front, counter-clockwise (90 = shooting out the LEFT side, -90 = the RIGHT side,
        // 180 = out the back). The turret turns RIGHT (clockwise) from HOME.
        // MEASURED Oct 25 (programming lead): at HOME the turret faces the robot's front (intake side).
        public static final double kHomeYawDegrees = 0.0;

        // ---- Limelight (rides on the turret, faces the same way as the shooter) ----
        // MEASURE: camera lens position relative to the turret pivot, measured with the turret at
        // HOME: forward along the shooter's direction, left of it, and height above the floor.
        // MEASURED Oct 25: about 5 in BEHIND the turret center (toward the back of the robot at
        // HOME), facing the same way as the shooter. Negative = behind.
        public static final double kCameraForwardOfPivot = -5.0 * 0.0254;
        public static final double kCameraLeftOfPivot = 0.0;
        // MEASURED Oct 25: the Limelight rides on the HOOD, so its height (and tilt) change with
        // the hood. Lens height: 20.75 in with the hood all the way down, 22.35 in all the way up.
        // The code interpolates by hood position (Gooba.kBottomPosition .. kTopPosition).
        public static final double kCameraHeightHoodDown = 20.75 * 0.0254;
        public static final double kCameraHeightHoodUp = 22.35 * 0.0254;
        // MEASURE: camera tilt, degrees UP from level.
        // MEASURED Oct 25: lens points UP, tilted back 25 degrees (phone level on top: -25 to -26).
        // Hood all the way UP: also 25 (measured Oct 25; a belt keeps the camera angle as the hood rises).
        public static final double kCameraPitchHoodDown = 25.0;
        public static final double kCameraPitchHoodUp = 25.0;   // measured Oct 25: a belt compensation mechanism keeps the camera at 25 deg as the hood rises
        // Skip vision while the hood (and camera) is moving faster than this (rotations/second).
        public static final double kMaxHoodSpeedForVision = 1.0;
        // Limelight's robot-space axes. Its web UI labels them "forward / right / up" and turns
        // counter-clockwise for positive yaw. If the vision position jumps when ONLY the turret
        // turns (robot still, tags in view), flip kLimelightYawCCW; if it mirrors left/right,
        // flip kLimelightSideIsRight. (Test in CLAUDE.md.)
        public static final boolean kLimelightSideIsRight = true;
        public static final boolean kLimelightYawCCW = true;

        // ---- Turret position control (aiming at an angle, not chasing tx) ----
        // Percent power per degree of error. 20 degrees off = 0.4 power.
        public static final double kTurretP = 0.02;
        public static final double kTurretD = 0.0008;
        // Feedforward: percent power per (degree per second) the turret must turn, so it keeps up
        // when the robot spins under it. Estimate: 36 motor rot per 270 deg, ~100 rot/s at full power.
        public static final double kTurretFF = 0.0013;
        public static final double kTurretMaxOutput = 0.8;
        // "On target" when within this many degrees and turning slower than kLockedMaxSpeed.
        public static final double kTurretToleranceDegrees = 1.5;
        public static final double kLockedMaxSpeedDegPerSec = 30.0;

        // ---- Ready to fire (Operator RT only feeds the shooter when all are true) ----
        public static final double kRpmTolerance = 150.0;   // to START firing
        public static final double kKeepFiringDegrees = 4.0; // once firing, keep feeding unless the turret is this far off
        public static final double kHoodTolerance = 0.12; // rotations

        // ---- Hub out of the turret's reach (the ~90 degree dead zone behind the wires) ----
        // The drivetrain turns the robot just enough to bring the hub into reach, but only while
        // the driver isn't using the rotate stick. rad/s per radian of error, and a cap.
        public static final double kHeadingAssistP = 3.0;
        public static final double kHeadingAssistMax = 2.5; // rad/s
        public static final double kDriverRotateOverride = 0.15; // right-stick deflection that takes over

        // ---- Shooting while moving ----
        // Aim at where the hub will be relative to the robot when the ball gets there
        // (robot velocity x time of flight). Time of flight comes from the shot table.
        public static final boolean kShootWhileMoving = true;

        // ---- Vision position (Limelight MegaTag) ----
        // Ignore vision while the turret (and camera) is swinging faster than this, or the robot
        // spins faster than this: the image and the turret angle don't line up then.
        public static final double kMaxTurretSpeedForVision = 120.0; // deg/s
        public static final double kMaxRobotSpinForVision = 2.5;     // rad/s
        // Ignore single-tag fixes from further away than this (meters): too noisy.
        public static final double kMaxSingleTagDistance = 4.5;
        // How much to trust a fix: standard deviation (m) for one tag at 1 m. Grows with
        // distance squared, shrinks with more tags.
        public static final double kVisionXYStdDevAt1m = 0.25;
        // Heading from vision (MegaTag1, 2+ tags) is only used to correct the gyro slowly.
        public static final double kVisionThetaStdDev = 0.6; // rad (UNUSED since Oct 25: vision no longer turns a known heading)
        public static final double kDisabledThetaStdDev = 0.15; // rad (UNUSED since Oct 25, see VisionLocalizer)

        // ---- Hub centers on the field (meters, WPILib blue-origin coordinates) ----
        // From the official 2026 REBUILT welded field layout (apriltag 2026.2.1): the middle of
        // each hub's 8 tags. The AndyMark field differs by about 1.4 cm.
        public static final double kRedHubX = 11.9155;
        public static final double kBlueHubX = 4.6255;
        public static final double kHubY = 4.0346;
    }
}
