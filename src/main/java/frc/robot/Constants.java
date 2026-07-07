package frc.robot;

import static edu.wpi.first.units.Units.*;
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

    public static final class Pneumatics {
        public static final int kPcmId = 25;
        public static final int kSol1Forward = 1;
        public static final int kSol1Reverse = 0;
        public static final int kSol2Forward = 2;
        public static final int kSol2Reverse = 3;
        public static final int kSol3Forward = 6;
        public static final int kSol3Reverse = 7;
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
        public static final double kIdleRPM = 1700.0;
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
        public static final int kMotorId = 13;
        public static final boolean kInverted = false;
        public static final double kForwardSpeed = 12.0; // volts
        public static final double kReverseSpeed = -6.0; // volts
    }

    public static final class ShooterIntake {
        public static final int kMotorId = 15;
        public static final boolean kInverted = true;
        public static final double kForwardSpeed = 2.0;
        public static final double kReverseSpeed = -0.6;
    }

    public static final class Turret {
        public static final int kMotorId = 14;
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
        public static final double kManualStep = 0.04;
    }

    public static final class GroundIntake {
        public static final int kMotorId = 20;
        public static final boolean kInverted = false;
        public static final double kIntakeSpeed = 0.75;
        public static final double kReverseSpeed = -0.75; // spit fuel back out
        public static final double kSupplyCurrentLimit = 40.0;
    }

    public static final class Climb {
        public static final int kMotorId = 21;
        public static final double kClimbSpeed = 0.20;
        public static final boolean kMotorInverted = false;
        public static final double kSupplyCurrentLimit = 60.0;
    }

    public static final class Limelight {
        // Camera is mounted 1.5 in right of the shooter centerline
        public static final double kHOffsetMeters = 1.5 * 0.0254;
        public static final int[] kValidTargetIds = {10, 18, 21, 26, 5, 2};

        // The Limelight is mounted SIDEWAYS, so tx measures the vertical angle
        // to the tag. These feed the trig in LimelightSubsystem.distanceToTarget().
        public static final double kMountAngleDegrees = 36.4;
        public static final double kLensHeightMeters = 0.0;
        public static final double kAprilTagHeightMeters = 1.12395;
    }
}
