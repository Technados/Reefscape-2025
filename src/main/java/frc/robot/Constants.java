// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  

  public static final class CoralSubsystemConstants {
    public static final int kElevatorMotorCanId = 10;
    public static final int kArmMotorCanId = 11;
    public static final int kIntakeMotorCanId = 9;
    public static final int kFrontIntakeMotorCanId = 13;

    public static final class ElevatorSetpoints {
      public static final int kFeederStation = 22;
      public static final int kLevel1 = 0;
      public static final int kLevel2 = 0;
      public static final int kLevel3 = 40;
      public static final int kLevelA = 80;
      public static final int kLevel4 = 148;
    }

    public static final class ArmSetpoints {
      public static final double kFeederStation = -1.25;
      public static final double kLevel1 = 0;
      public static final double kLevel2 = 16;
      public static final double kLevel3 = 17.75;
      public static final double kLevel4 = 18.6;
      public static final double kKnockBack = 20.5;
      public static final double kAlgaeLow = 6.5;
      public static final double kAlgaeHigh = 6.5;
      public static final double kAlgaeScore1 = 25;
      public static final double kAlgaeScore2 = 14;
    }

    public static final double kArmDegreesPerEncoderTick = 8.26; // based on 20:1 planetary AND a 22t --> 48t chain reduction

    public static final class IntakeSetpoints {
      public static final double kForward = -0.5;
      public static final double kReverse = 0.90;
      public static final double kHold = -0.02; // adjust as needed
    }

    public static final class FrontIntakeSetpoints {
      public static final double kForward = 0.40;
      public static final double kReverse = -0.40;
    }

    // Gravity Feedforward
    public static final double kArmGravityFF = 0.7; // Arm feedforward (adjust as tuned)
    public static final double kArmGravityFFMin = 0.1;
    public static final double kElevatorGravityFF = 0.20; // Elevator feedforward (adjust as tuned)

    // Slow zone behavior
    public static final double kElevatorSlowZone = 5.0; // Number of ticks from limit to start slowing
    public static final double kElevatorSlowSpeedFactor = 0.5; // 40% of speed when inside slow zone


  }

  public static final class AlgaeSubsystemConstants {
    public static final int kIntakeMotorCanId = 12;

    public static final class IntakeSetpoints{
      public static final double kForward = -0.60;
      public static final double kReverse = 0.85;
      public static final double kHold = -0.20;
    }
  }
  public static final class ClimberSubsystemConstants {
    public static final int kClimbMotorCanId = 14;
    public static final int kClimbArmMotorCanId = 15;

    public static final class ArmSetpoints {
      public static final double kStow = 18.5;
      public static final double kHold = 11.5;
      public static final double kDown = 0;
    }

    public static final class ClimbSetpoints {
      public static final double kForward = 0.5;
      public static final double kReverse = -0.5;
      public static final double kHold = 0.25;
    }
  }

  public static final class AprilTagIDs {
    // 🚨 Reef AprilTag IDs (Red Alliance Perspective, Counter-Clockwise, Faces A --> F)
    public static final int A = 7;
    public static final int B = 8;
    public static final int C = 9;
    public static final int D = 10;
    public static final int E = 11;
    public static final int F = 6;
    public static final int BLUE_A = 18;
    public static final int BLUE_B = 17;
    public static final int BLUE_C = 22;
    public static final int BLUE_D = 21;
    public static final int BLUE_E = 20;
    public static final int BLUE_F = 19;

    // 🚨 Substation AprilTag IDs
    public static final int LEFT_SUBSTATION_RED = 6;
    public static final int RIGHT_SUBSTATION_RED = 8;
    public static final int LEFT_SUBSTATION_BLUE = 19;
    public static final int RIGHT_SUBSTATION_BLUE = 17;

}
    // Desired distances for alignment (to be fine-tuned)
    public static final class DesiredDistances {
      public static final double REEF_SCORING = 0.0; // Close to reef
      public static final double SUBSTATION_PICKUP = 10.0; // ~15ft from reef
  }
          // Substation Heading Angles (from driver station perspective)
          public static final double LEFT_SUBSTATION_ANGLE = 125.0;
          public static final double RIGHT_SUBSTATION_ANGLE = 235.0;

public static final class LimelightPID {
  // 🎯 PID Gains for Reef Targeting

  public static final double kP_turn = 0.15;   // Previously 0.1
  public static final double kI_turn = 0.000;
  public static final double kD_turn = 0.0;   // New (small D gain)

  public static final double kP_distance = 0.2; // Previously 0.15
  public static final double kI_distance = 0.0;
  public static final double kD_distance = 0.0; // New (small D gain)

  public static final double kP_strafe = 0.15;  // Previously 0.15
  public static final double kI_strafe = 0.0;
  public static final double kD_strafe = 0.0005; // New (small D gain)

}


  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    public static final double kSlowSpeedFactor = 0.20; // Slow mode speed factor (40% of normal speed)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(23.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kFrontRightDrivingCanId = 2;
    public static final int kRearRightDrivingCanId = 4;

    public static final int kFrontLeftTurningCanId = 5;
    public static final int kRearLeftTurningCanId = 7;
    public static final int kFrontRightTurningCanId = 6;
    public static final int kRearRightTurningCanId = 8;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.1;
    public static final double kTriggerButtonThreshold = 0.2;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class SimulationRobotConstants {
    public static final double kPixelsPerMeter = 20;

    public static final double kElevatorGearing = 25; // 25:1
    public static final double kCarriageMass =
        4.3 + 3.15 + 0.151; // Kg, arm + elevator stage + chain
    public static final double kElevatorDrumRadius = 0.0328 / 2.0; // m
    public static final double kMinElevatorHeightMeters = 0.922; // m
    public static final double kMaxElevatorHeightMeters = 1.62; // m

    public static final double kArmReduction = 60; // 60:1
    public static final double kArmLength = 0.433; // m
    public static final double kArmMass = 4.3; // Kg
    public static final double kMinAngleRads =
        Units.degreesToRadians(-50.1); // -50.1 deg from horiz
    public static final double kMaxAngleRads =
        Units.degreesToRadians(40.9 + 180); // 40.9 deg from horiz

    public static final double kIntakeReduction = 135; // 135:1
    public static final double kIntakeLength = 0.4032262; // m
    public static final double kIntakeMass = 5.8738; // Kg
    public static final double kIntakeMinAngleRads = Units.degreesToRadians(80);
    public static final double kIntakeMaxAngleRads = Units.degreesToRadians(180);
    public static final double kIntakeShortBarLength = 0.1524;
    public static final double kIntakeLongBarLength = 0.3048;
    public static final double kIntakeBarAngleRads = Units.degreesToRadians(-60);
  }
}
