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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import frc.robot.Constants;
import frc.robot.Constants.AprilTagIDs;

import frc.robot.Constants.DriveConstants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;



public class DriveSubsystem extends SubsystemBase {

  private final PIDController limelightTurnPID = new PIDController(
    Constants.LimelightPID.kP_turn, 
    Constants.LimelightPID.kI_turn, 
    Constants.LimelightPID.kD_turn
);

private final PIDController limelightStrafePID = new PIDController(
    Constants.LimelightPID.kP_strafe, 
    Constants.LimelightPID.kI_strafe, 
    Constants.LimelightPID.kD_strafe
);

private final PIDController limelightDistancePID = new PIDController(
    Constants.LimelightPID.kP_distance, 
    Constants.LimelightPID.kI_distance, 
    Constants.LimelightPID.kD_distance
);

private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
private final AHRS m_gyro = new AHRS(SerialPort.Port.kUSB);

public enum Alignment {
    LEFT,
    RIGHT,
    CENTER
}
private final CoralSubsystem coralSubsystem;

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
  //private final AHRS m_gyro = new AHRS(SerialPort.Port.kUSB);
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

      private final LEDSubsystem ledSubsystem;
      private boolean hasFlashedEndgame = false;


      public DriveSubsystem(CoralSubsystem coralSubsystem, LEDSubsystem ledSubsystem) {
  
        this.ledSubsystem = ledSubsystem;
      
        this.coralSubsystem = coralSubsystem;
                  // set limelight alignment tolerance
                  limelightTurnPID.setTolerance(3);
                  limelightDistancePID.setTolerance(0.75);
                  limelightStrafePID.setTolerance(0.5);

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
    if (!m_gyro.isConnected()) {
      ledSubsystem.setPattern(0.61); // 🔴 Red if gyro offline
  }
  


    
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });

        if (edu.wpi.first.wpilibj.DriverStation.isTeleopEnabled() &&
    edu.wpi.first.wpilibj.DriverStation.getMatchTime() <= 30.0 &&
    !hasFlashedEndgame) {

    ledSubsystem.flashPattern(0.87, 2.0); // 🔵 Blue flash for 2 seconds
    hasFlashedEndgame = true;
}

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

  private boolean slowMode = false; // Boolean flag to track slow mode

public void setSlowMode(boolean enable) {
    slowMode = enable;
}

// Threshold to auto-enable slow mode when elevator is high
private static final double kElevatorSlowModeThreshold = 40.0; // <-- Set this to whatever height makes sense (encoder ticks)

/**
 * Check if we need to enable slow mode based on elevator height or driver input.
 * @param manualSlowMode true if driver is holding right bumper
 */
public void updateDriveSlowMode(boolean manualSlowMode) {
    // Check if elevator is above threshold
    boolean elevatorHigh = coralSubsystem.getElevatorHeight() > kElevatorSlowModeThreshold;

    // Enable slow mode if either driver asks for it or elevator is high
    setSlowMode(manualSlowMode || elevatorHigh);
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
    SmartDashboard.putBoolean("Slow Mode", slowMode);
    // Convert the commanded speeds into the correct units for the drivetrain
    // Apply speed reduction if slow mode is active
    // Apply speed reduction factor
    double speedFactor = slowMode ? DriveConstants.kSlowSpeedFactor : 1.0;

    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond * speedFactor;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond * speedFactor;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed * speedFactor; // Ensure rotation is also scaled

    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    -xSpeedDelivered,
                    -ySpeedDelivered,
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

  public Command moveFixedDistance(double xMeters, double yMeters) {
    return new InstantCommand(() -> {
        drive(xMeters, yMeters, 0, false); // Move in robot-relative space
    }, this).andThen(new InstantCommand(() -> drive(0, 0, 0, false), this)); // Stop after movement
}



  public void stopMovement() {
    drive(0, 0, 0, true); // Stop all movement
}
  
/**
 * Align to the reef for scoring or algae removal.
 */

/**
 * Align to the reef (or center algae) using AprilTag + Limelight data.
 */
public void alignToReef(Alignment alignment) {
  // 🔹 Set pipeline based on alignment
  int pipeline = switch (alignment) {
      case LEFT -> 0;
      case RIGHT -> 1;
      case CENTER -> 2;
  };
  limelight.getEntry("pipeline").setNumber(pipeline);

  double tx = limelight.getEntry("tx").getDouble(0.0);
  double ty = limelight.getEntry("ty").getDouble(0.0);
  int tagID = (int) limelight.getEntry("tid").getDouble(-1);

  // 🔹 Validate tag detection
  if (tagID == -1) {
      stopMovement();
      setSlowMode(false);
      SmartDashboard.putString("ReefAlign Status", "No AprilTag Detected");
      return;
  }

  double targetHeading = getReefHeading(tagID);
  if (Double.isNaN(targetHeading)) {
      stopMovement();
      setSlowMode(false);
      SmartDashboard.putString("ReefAlign Status", "Invalid Target Heading");
      return;
  }

  // 🔹 Enable slow mode
  setSlowMode(true);

  // 🔹 Step 1: Rotate first
  double turnPower = limelightTurnPID.calculate(getHeading(), targetHeading);
  boolean turnComplete = limelightTurnPID.atSetpoint();

  if (!turnComplete) {
      drive(0, 0, turnPower, false);
      SmartDashboard.putString("ReefAlign Status", "Turning to Target");
      return;
  }

  // 🔹 Step 2: Simultaneous tx + ty movement
  double forwardPower = limelightDistancePID.calculate(ty, Constants.DesiredDistances.REEF_SCORING);
  double strafePower = -limelightStrafePID.calculate(tx, 0.0);

  // Apply deadbands to eliminate twitching
  if (Math.abs(forwardPower) < 0.02) forwardPower = 0;
  if (Math.abs(strafePower) < 0.02) strafePower = 0;

  boolean driveComplete = limelightDistancePID.atSetpoint();
  boolean strafeComplete = limelightStrafePID.atSetpoint();

  if (!driveComplete || !strafeComplete) {
      drive(forwardPower, strafePower, 0, false);
      SmartDashboard.putString("ReefAlign Status", "Driving to Target");
      SmartDashboard.putNumber("tx", tx);
      SmartDashboard.putNumber("ty", ty);
      SmartDashboard.putBoolean("TX Done", strafeComplete);
      SmartDashboard.putBoolean("TY Done", driveComplete);
      return;
  }

  // 🔹 Final Step: Stop + restore normal mode
  stopMovement();
  setSlowMode(false);
  drive(0, 0, 0, true); // Return to field-relative driving
  ledSubsystem.flashPattern(0.91, 1.5); // Flash purple

  SmartDashboard.putString("ReefAlign Status", "Alignment Complete");
}
////////////////////////////////////////////////////////////////////////////////////////////////////
/// old reef align command - keeping temp until new one tested
/// 
// private double lastValidTargetTime = 0; // Track when Limelight last updated

// public void alignToReef(Alignment alignment) {
//     int pipeline = (alignment == Alignment.LEFT) ? 0 : 1;
//     limelight.getEntry("pipeline").setNumber(pipeline);

//     double tx = limelight.getEntry("tx").getDouble(0.0);
//     double ty = limelight.getEntry("ty").getDouble(0.0);
//     int tagID = (int) limelight.getEntry("tid").getDouble(-1);

//     // Ensure valid tag is detected
//     if (tagID == -1) {
//         stopMovement();
//         setSlowMode(false);
//         SmartDashboard.putString("ReefAlign Status", "No AprilTag Detected");
//         return;
//     }

//     // Ensure data is fresh (no stale readings)
//     // lastValidTargetTime = Timer.getFPGATimestamp();
//     // double currentTime = Timer.getFPGATimestamp();
//     // if (currentTime - lastValidTargetTime > 0.5) { // If Limelight data is stale
//     //     stopMovement();
//     //     setSlowMode(false);
//     //     SmartDashboard.putString("ReefAlign Status", "Stale Data - Not Aligning");
//     //     return;
//     // }

//     double targetHeading = getReefHeading(tagID);
//     if (Double.isNaN(targetHeading)) {
//         stopMovement();
//         setSlowMode(false);
//         SmartDashboard.putString("ReefAlign Status", "Invalid Target Heading");
//         return;
//     }

//     // ✅ Enable Slow Drive Mode
//     setSlowMode(true);

//     // 🔹 Step 1: Align Heading First
//     double turnPower = limelightTurnPID.calculate(getHeading(), targetHeading);
//     boolean turnComplete = limelightTurnPID.atSetpoint();

//     if (!turnComplete) {
//         drive(0, 0, turnPower, false);
//         SmartDashboard.putString("ReefAlign Status", "Turning to Target");
//         return;
//     } else {
//         drive(0, 0, 0, false); // Stop turning
//     }

//     // 🔹 Step 3: Move Forward to Reach the Reef
//     double drivePower = limelightDistancePID.calculate(ty, 0);
//     boolean driveComplete = limelightDistancePID.atSetpoint();
    
//     if (!driveComplete) {
//         drive(drivePower, 0, 0, false);
//         SmartDashboard.putString("ReefAlign Status", "Driving Forward");
//         return;
//      }

//      // 🔹 Step 2: Strafe Until Crosshair is Centered
//     double strafePower = -limelightStrafePID.calculate(tx, 0.0);
//     boolean strafeComplete = limelightStrafePID.atSetpoint();

//     // // ✅ Apply Deadband to prevent unnecessary micro-movements
//     if (Math.abs(strafePower) < 0.00) {
//         strafePower = 0;
//     }

//     if (!strafeComplete) {
//         drive(0, strafePower, 0, false);
//         SmartDashboard.putString("ReefAlign Status", "Strafing");
//         return;
//     }

//     // 🔹 Final Stop Condition - All Steps Complete
//     if (turnComplete && driveComplete && strafeComplete) {
//         stopMovement();
//         setSlowMode(false);
//         drive(0, 0, 0, true); // ✅ Ensures next movement is field-relative
//         SmartDashboard.putString("ReefAlign Status", "Alignment Complete");
//     }
// }
/////////////////////////////////////////////////////////////////////////////////////////////////

  private double getReefHeading(int tagID) {
    switch (tagID) {
        case AprilTagIDs.A: return 0.0;
        case AprilTagIDs.B: return 60.0;
        case AprilTagIDs.C: return 120.0;
        case AprilTagIDs.D: return (getHeading() > 0 ? 180.0 : -180.0);  // Ensures shortest turn
        case AprilTagIDs.E: return -120.0;
        case AprilTagIDs.F: return -60.0;
        case AprilTagIDs.BLUE_A: return 0.0;
        case AprilTagIDs.BLUE_B: return 60.0;
        case AprilTagIDs.BLUE_C: return 120.0;
        case AprilTagIDs.BLUE_D: return (getHeading() > 0 ? 180.0 : -180.0);  // Same for blue alliance
        case AprilTagIDs.BLUE_E: return -120.0;
        case AprilTagIDs.BLUE_F: return -60.0;
        default: return Double.NaN;
    }
}
  // public void alignToSubstation() {
  //     int tagID = (int) limelight.getEntry("tid").getDouble(-1);
  //     double targetHeading = (tagID == 6 || tagID == 19) ? 125.0 : (tagID == 17 || tagID == 8) ? -125.0 : Double.NaN;
  //     if (targetHeading == Double.NaN) return;

  //     double tx = limelight.getEntry("tx").getDouble(0.0);
  //     double ty = limelight.getEntry("ty").getDouble(0.0);

  //     double turnPower = limelightTurnPID.calculate(getHeading(), targetHeading);
  //     double drivePower = limelightDistancePID.calculate(ty, Constants.DesiredDistances.SUBSTATION_PICKUP);

  //     if (!limelightTurnPID.atSetpoint()) {
  //         drive(0, 0, turnPower, false);
  //     } else if (!limelightDistancePID.atSetpoint()) {
  //         drive(drivePower, 0, 0, false);
  //     } else {
  //         drive(0, 0, 0, false);
  //     }
  // }

  public Command alignToReefCommand(Alignment alignment) {
      return new RunCommand(() -> alignToReef(alignment), this);
  }

  // public Command alignToSubstationCommand() {
  //     return new RunCommand(this::alignToSubstation, this);
  // }

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
    return Math.IEEEremainder(-m_gyro.getAngle()*(DriveConstants.kGyroReversed ? -1.0 : 1.0), 360);
    //return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
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
