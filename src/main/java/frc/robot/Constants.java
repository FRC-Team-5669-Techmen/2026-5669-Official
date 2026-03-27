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
        public static final double kfastTargetRPM = 7000.0; 
        public static final double kslowTargetRPM = 2500.0; 
        public static final double kReverseRPM = -1000.0;   
        public static final double kTestingRPM = 1500.0;    
        public static final double kMaxRPM = 3100.0; 
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
        public static final double kForwardSpeed = 12.0; 
        public static final double kReverseSpeed = -6.0; 
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
        public final static double kmountAngleDegrees = 36.4;
        public final static double klensheightmeters = 0.0;
        public final static double kaprilTagHeightMeters = 1.12395;
    }

    public static final class GroundIntake {
        public static final int kMotorId = 20; 
        public static final boolean kInverted = false; 
        public static final double kIntakeSpeed = 0.75; 
        
        // NEW: Reverse speed for spitting notes back out
        public static final double kReverseSpeed = -0.75; 
        
        public static final double kSupplyCurrentLimit = 40.0;
    }

    public static final class Climb {
        public static final int kMotorId = 21; 
        public static final double kClimbSpeed = 0.20; 
        public static final boolean kMotorInverted = false; 
        public static final double kSupplyCurrentLimit = 60.0; 
    }

    public static final class Limelight {
        public static final double kHOffsetMeters = 1.5 * 0.0254;
        public static final int[] kValidTargetIds = {10, 18, 21, 26, 5, 2}; 
    }
}