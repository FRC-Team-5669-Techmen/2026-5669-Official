package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.AngularVelocity;

public final class Constants {

    public static final class Operator {
        public static final int kDriverControllerPort = 0;
        
        public static final double kDeadband = 0.1;
        public static final double kRotationalDeadband = 0.1;

        public static final AngularVelocity kMaxAngularRate = RotationsPerSecond.of(0.75);
    }

    public static final class Shooter {
        public static final int kLeaderId = 16;
        public static final int kFollowerId = 17;

        public static final double kTargetRPM = 5000.0;

        public static final double kP = -0.11;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kV = 0.0;

        public static final double kVoltageRampPeriod = 100.0;
        public static final double kDutyCycleRampPeriod = 100.0;
    }

    public static final class Turret {
        public static final int kMotorId = 61;
        
        public static final double kSpeedMultiplier = 0.25;

        public static final double kP = 0.03;
        public static final double kI = 0.0;
        public static final double kD = 0.002;
        
        public static final double kToleranceDegrees = 0.5;
        public static final double kMaxOutput = 0.3;
    }

    public static final class Auton {
        public static final double kDriveSpeed = 0.5;
        public static final double kTimeoutSeconds = 5.0;
    }

    public static final class Sim {
        public static final double kLoopPeriod = 0.004;
    }
}