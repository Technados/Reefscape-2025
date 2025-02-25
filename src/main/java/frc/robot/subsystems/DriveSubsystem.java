// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import frc.robot.Constants;
import frc.robot.Constants.AprilTagIDs;
import frc.robot.Constants.CoralSubsystemConstants.IntakeSetpoints;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class DriveSubsystem extends SubsystemBase {

  private static final double ALIGN_THRESHOLD = 1.0;
  private static final double RANGE_THRESHOLD = 2.0; 

  // 🎯 Strafe PID Controller (for left/right alignment with reef face)
private final PIDController limelightStrafePID = new PIDController(
  Constants.LimelightPID.kP_strafe, 
  Constants.LimelightPID.kI_strafe, 
  Constants.LimelightPID.kD_strafe
);
private final PIDController limelightTurnPID = new PIDController(
    Constants.LimelightPID.kP_turn, 
    Constants.LimelightPID.kI_turn, 
    Constants.LimelightPID.kD_turn
);

private final PIDController limelightDistancePID = new PIDController(
    Constants.LimelightPID.kP_distance, 
    Constants.LimelightPID.kI_distance, 
    Constants.LimelightPID.kD_distance
);


  

  private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
  public enum Alignment {
    LEFT,
    RIGHT,
    CENTER
    
}

  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft =
      new MAXSwerveModule(
          DriveConstants.kFrontLeftDrivingCanId,
          DriveConstants.kFrontLeftTurningCanId,
          DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight =
      new MAXSwerveModule(
          DriveConstants.kFrontRightDrivingCanId,
          DriveConstants.kFrontRightTurningCanId,
          DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft =
      new MAXSwerveModule(
          DriveConstants.kRearLeftDrivingCanId,
          DriveConstants.kRearLeftTurningCanId,
          DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight =
      new MAXSwerveModule(
          DriveConstants.kRearRightDrivingCanId,
          DriveConstants.kRearRightTurningCanId,
          DriveConstants.kBackRightChassisAngularOffset);

          
  // The gyro sensor
  // Next line is gyro setup for NavX-2 Micro gyro from Kauai Labs
  //// Additional change: since using NavX-2 gyro, all getAngle calls in the drive
  // sub system had to be chnaged to negative values
  private final AHRS m_gyro = new AHRS(SerialPort.Port.kUSB);
    // The NavX gyro is used to track the robot's orientation on the field.


  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          Rotation2d.fromDegrees(-m_gyro.getAngle()),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });

      // PathPlanner RobotConfig
      private RobotConfig config;

      public DriveSubsystem() {

          // set limelight alignment tolerance
          limelightTurnPID.setTolerance(ALIGN_THRESHOLD);
          limelightDistancePID.setTolerance(RANGE_THRESHOLD);

          // Load RobotConfig
          try {
              config = RobotConfig.fromGUISettings();
          } catch (Exception e) {
              e.printStackTrace();
          }
  
          // Configure AutoBuilder at the end
          configureAutoBuilder();
      }
  
      private void configureAutoBuilder() {
          AutoBuilder.configure(
              this::getPose, // Robot pose supplier
              this::resetOdometry, // Method to reset odometry
              this::getChassisSpeeds, // Robot-relative ChassisSpeeds supplier
              (speeds, feedforwards) -> driveRobotRelative(speeds), // Drive robot
              new PPHolonomicDriveController( // Holonomic controller
                  new PIDConstants(8.5, 0.0, 0.11), // Translation PID
                  new PIDConstants(13.0, 0.0, 0.4)  // Rotation PID
              ),
              config, // RobotConfig loaded from PathPlanner GUI
              () -> {
                  // Flip paths for red alliance
                  var alliance = DriverStation.getAlliance();
                  return alliance.orElse(DriverStation.Alliance.Blue) != DriverStation.Alliance.Blue;
              },
              this // Subsystem requirements
          );
      }

      


  @Override
  public void periodic() {

    SmartDashboard.putNumber("Gyro", getHeading()); // returns the heading of the robot and sends to dashboard


    
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    SmartDashboard.putBoolean("Field Relative", fieldRelative);
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotDelivered,
                    Rotation2d.fromDegrees(-m_gyro.getAngle()))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
    
    //Show Joystick input on DashBoard
    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("rot", rot);
    
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
}

public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    });
}

  /** Sets the wheels into an X formation to prevent movement. */
  public Command setXCommand() {
    return this.run(
        () -> {
          m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
          m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
          m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
          m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        });
  }
  

/**
     * Align to the reef for scoring or algae removal.
     */
    public void alignToReef(Alignment alignment) {
    limelight.getEntry("pipeline").setNumber(0); // Use Reef Scoring Pipeline

    double tx = limelight.getEntry("tx").getDouble(0.0);
    double ty = limelight.getEntry("ty").getDouble(0.0);
    double ta = limelight.getEntry("ta").getDouble(0.0);
    int tagID = (int) limelight.getEntry("tid").getDouble(-1);

    double targetHeading = 0.0;
    double strafeOffset = 0.0;

    // 🎯 Reef Face Headings based on AprilTag ID
    switch (tagID) {
        case AprilTagIDs.A: targetHeading = 0.0; break;   // ID 7
        case AprilTagIDs.B: targetHeading = 60.0; break;   // ID 8
        case AprilTagIDs.C: targetHeading = 120.0; break; // ID 9
        case AprilTagIDs.D: targetHeading = 180.0; break;  // ID 10
        case AprilTagIDs.E: targetHeading = -120.0; break; // ID 11
        case AprilTagIDs.F: targetHeading = -60.0; break; // ID 6
        default: return; // No valid tag detected
    }

    // Adjust for Left/Right Scoring Positions
    if (alignment == Alignment.LEFT) {
        strafeOffset = -0.2;  // Shift left ~6 inches
    } else if (alignment == Alignment.RIGHT) {
        strafeOffset = 0.2;   // Shift right ~6 inches
    } else {
        strafeOffset = 0.0;   // Center Alignment for Algae Removal
    }

    // 🎯 PID Control for Alignment
    double turnPower = limelightTurnPID.calculate(getHeading(), targetHeading);
    double drivePower = limelightDistancePID.calculate(ty, Constants.DesiredDistances.REEF_SCORING);
    double strafePower = limelightStrafePID.calculate(tx, strafeOffset);

    // 🎯 Execute Alignment in Sequence: Turn → Strafe → Drive
    if (!limelightTurnPID.atSetpoint()) {
        drive(0, 0, turnPower, false);
    } else if (!limelightStrafePID.atSetpoint()) {
        drive(0, strafePower, 0, false);
    } else if (!limelightDistancePID.atSetpoint()) {
        drive(drivePower, 0, 0, false);
    } else {
        drive(0, 0, 0, false);
    }
}
public Command alignToReefCommand(Alignment alignment) {
  return new RunCommand(() -> alignToReef(alignment), this);
}

/**
   * Align to the correct substation (left or right) for retrieving a game piece.
   */
  public void alignToSubstation() {
    int tagID = (int) limelight.getEntry("tid").getDouble(-1); // Get detected AprilTag ID

    // Determine which pipeline to use
    if (tagID == 6 || tagID == 19) {
        limelight.getEntry("pipeline").setNumber(1); // Left Substation Pipeline
    } else if (tagID == 17 || tagID == 8) {
        limelight.getEntry("pipeline").setNumber(2); // Right Substation Pipeline
    } else {
        return; // No valid tag detected
    }

    double tx = limelight.getEntry("tx").getDouble(0.0);
    double ty = limelight.getEntry("ty").getDouble(0.0);

    double turnPower = limelightTurnPID.calculate(tx, 0);
    double drivePower = limelightDistancePID.calculate(ty, Constants.DesiredDistances.SUBSTATION_PICKUP);

    if (!limelightTurnPID.atSetpoint()) {
        drive(0, 0, turnPower, false);
    } else if (!limelightDistancePID.atSetpoint()) {
        drive(drivePower, 0, 0, false);
    } else {
        drive(0, 0, 0, false);
    }
}

public Command alignToSubstationCommand() {
    return new RunCommand(this::alignToSubstation, this);
}



  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public Command zeroHeadingCommand() {
    return this.runOnce(() -> m_gyro.reset());
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }


}
