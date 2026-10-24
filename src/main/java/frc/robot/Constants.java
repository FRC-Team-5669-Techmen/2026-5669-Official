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
        public static final double kFastTargetRPM = 7000.0;
        public static final double kSlowTargetRPM = 2500.0;
        public static final double kReverseRPM = -1000.0;
        public static final double kTestingRPM = 1500.0;
        public static final double kMaxRPM = 3100.0; //7000
        // Driver D-pad Left idle speed. 2000 RPM = about 33.3 rotations per second
        // (the shooter code divides by 60 for you).
        public static final double kIdleRPM = 2000.0;
        public static final double kDecelerateStep = 60.0;
        public static final double kIdleAccelerateStep = 50.0;
        public static final double kP = 0.11;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kV = 0.12;
        public static final double kSupplyCurrentLimit = 80.0;
        public static final double kStatorCurrentLimit = 160.0;
        public static final double kVoltageRampPeriod = 0.0;
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
        public static final double kForwardSpeed = 2.0;
        public static final double kReverseSpeed = -0.6;
    }

    public static final class Turret {
        public static final int kMotorId = 14; // Tuner X name: "AngleShooter"
        public static final double kSpeedMultiplier = 1.0;
        public static final double kManualJogSpeed = 0.2;
        public static final double kSweepSpeed = 1.0;
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

    // RETIRED (Oct 2026): the robot no longer has a hood motor. The hood code is kept
    // but never created in RobotContainer. ID 51 is empty on the CAN bus; if a hood
    // comes back, give it a free ID in Tuner X and update kMotorId.
    public static final class Gooba {
        public static final int kMotorId = 51;
        public static final double kP = 2.4;
        public static final double kI = 0.0;
        public static final double kD = 0.1;
        public static final double kCruiseVelocity = 80.0;
        public static final double kAcceleration = 160.0;
        public static final double kSupplyCurrentLimit = 40.0;
        public static final double kPositionStowed = 0.0;
        public static final double kPositionDeployed = 3.5;
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
        public static final double kRollerVolts = 6.0;  // spin speed while RT is held (volts)
        public static final double kStatorCurrentLimit = 40.0;  // amps
    }

    public static final class Limelight {
        public static final int[] kValidTargetIds = {10, 18, 21, 26, 5, 2};

        // Offseason remount: camera is horizontal and centered on the shooter,
        // so tx aims the turret directly and ty feeds the distance trig in
        // LimelightSubsystem.distanceToTarget().
        // TODO: re-measure the mount angle and lens height on the new mount.
        public static final double kMountAngleDegrees = 36.4;
        public static final double kLensHeightMeters = 0.0;
        public static final double kAprilTagHeightMeters = 1.12395;
    }
}
