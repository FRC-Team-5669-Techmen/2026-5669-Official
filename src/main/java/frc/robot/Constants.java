package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.AngularVelocity;

public final class Constants {

    public static final class Operator {
        public static final int kDriverControllerPort = 0;
        
        public static final double kDeadband = 0.0;
        public static final double kRotationalDeadband = 0.0;

        public static final AngularVelocity kMaxAngularRate = RotationsPerSecond.of(0.75);
    }

    // --- SHOOTER (KEPT EXACTLY AS UPLOADED) ---
    public static final class Shooter {
        public static final int kLeaderId = 16;
        public static final int kFollowerId = 17;
        
        public static final boolean kLeaderInverted = false; 

        public static final double kfastTargetRPM = 5000.0; 
        public static final double kslowTargetRPM = 2500.0;
        public static final double kReverseRPM = -1000.0; 
        
        public static final double kP = -0.11;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kV = 0.0;

        public static final double kVoltageRampPeriod = 100.0;
        public static final double kDutyCycleRampPeriod = 100.0;
    }

    public static final class Index {
        public static final int kMotorId = 18; 
        public static final boolean kInverted = false; 

        public static final double kForwardSpeed = 0.5; 
        public static final double kReverseSpeed = -0.5; 
    }

    public static final class ShooterIntake {
        public static final int kMotorId = 19; 
        public static final boolean kInverted = false; 

        public static final double kForwardSpeed = 0.6; 
        public static final double kReverseSpeed = -0.6; 
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

    // --- NEW GOOBA CONSTANTS (Kraken X44) ---
    public static final class Gooba {
        public static final int kMotorId = 20; // [FIXME] Verify this ID
        
        // Motion Magic (Servo-like Position Control)
        public static final double kP = 2.4; 
        public static final double kI = 0.0;
        public static final double kD = 0.1;
        
        public static final double kCruiseVelocity = 80.0; // RPS
        public static final double kAcceleration = 160.0; // RPS/s
        
        // Safety for X44 (Smaller than X60)
        public static final double kSupplyCurrentLimit = 40.0; 
        
        // Positions (Rotations)
        public static final double kPositionStowed = 0.0;
        public static final double kPositionDeployed = 5.0; // [FIXME] Tune this value!
    }
}