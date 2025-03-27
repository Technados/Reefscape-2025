package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CoralSubsystemConstants;
import frc.robot.Constants.CoralSubsystemConstants.ArmSetpoints;
import frc.robot.Constants.CoralSubsystemConstants.ElevatorSetpoints;
import frc.robot.Constants.CoralSubsystemConstants.FrontIntakeSetpoints;
import frc.robot.Constants.CoralSubsystemConstants.IntakeSetpoints;
//import edu.wpi.first.wpilibj.DigitalInput;
//import frc.robot.subsystems.LEDSubsystem;

public class CoralSubsystem extends SubsystemBase {
  /** Subsystem-wide setpoints */
  public enum Setpoint {
    kFeederStation,
    kLevel1,
    kLevel2,
    kLevel3,
    kLevel4,
    kKnockBack,
    kAlgaeLow,
    kAlgaeHigh,
    kAlgaeScore1,
    kAlgaeScore2;
  }


  // Initialize arm SPARK. We will use MAXMotion position control for the arm, so we also need to
  // initialize the closed loop controller and encoder.
  private SparkMax armMotor =
      new SparkMax(CoralSubsystemConstants.kArmMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController armController = armMotor.getClosedLoopController();
  private RelativeEncoder armEncoder = armMotor.getEncoder();

  // Initialize elevator SPARK. We will use MAXMotion position control for the elevator, so we also
  // need to initialize the closed loop controller and encoder.
  private SparkMax elevatorMotor =
      new SparkMax(CoralSubsystemConstants.kElevatorMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController elevatorClosedLoopController =
      elevatorMotor.getClosedLoopController();
  private RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

  // Initialize intake SPARK. We will use open loop control for this so we don't need a closed loop
  // controller like above.
  private SparkFlex intakeMotor =
      new SparkFlex(CoralSubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);

   private SparkMax frontIntakeMotor =
      new SparkMax(CoralSubsystemConstants.kFrontIntakeMotorCanId, MotorType.kBrushless);

  // Member variables for subsystem state management
  private boolean wasResetByButton = false;
  //private boolean wasResetByLimit = false;
  private double armCurrentTarget = ArmSetpoints.kLevel1;
  private double elevatorCurrentTarget = ElevatorSetpoints.kLevel1;
  
  public double getElevatorHeight() {
    return elevatorEncoder.getPosition(); // returns encoder ticks
}


  // Elevator Limit Switch
  //private DigitalInput elevatorLimitSwitch = new DigitalInput(6);

  // safe elevator slow down zone near limits for dynamic speed reduction approaching limits
  // private static final double kElevatorSlowZone = 10.0; //encoder ticks before top/bottom to start slow zone 
  // private static final double kElevatorSlowSpeedFactor = 0.4; // 40% of full speed when near limit
  private final LEDSubsystem ledSubsystem;

  public CoralSubsystem(LEDSubsystem ledSubsystem) {
      this.ledSubsystem = ledSubsystem;
  
    /*
     * Apply the appropriate configurations to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK to a known state. This
     * is useful in case the SPARK is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    armMotor.configure(
        Configs.CoralSubsystem.armConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    elevatorMotor.configure(
        Configs.CoralSubsystem.elevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    intakeMotor.configure(
        Configs.CoralSubsystem.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    frontIntakeMotor.configure(
          Configs.CoralSubsystem.frontIntakeConfig,
          ResetMode.kResetSafeParameters,
          PersistMode.kPersistParameters);
   

    // Zero arm and elevator encoders on initialization
    armEncoder.setPosition(0);
    elevatorEncoder.setPosition(0);
    applyNormalArmConfig(); // Ensure arm starts in normal speed mode
    

  }

/**
 * Drive the arm and elevator motors to their respective setpoints. 
 * Applies smooth slow zone near both elevator limits, adds gravity feedforward,
 * and maintains arm/elevator sequencing logic.
 */
private void moveToSetpoint() {
  // ========================
  // Arm Feedforward Handling
  // ========================
  // Convert arm encoder to degrees and apply 90-degree offset for proper cos(angle)
  double armAngleDegrees = armEncoder.getPosition() * CoralSubsystemConstants.kArmDegreesPerEncoderTick;
  double armAngleRadians = Math.toRadians(armAngleDegrees - 90); // Offset so arm hanging down is cos(90)=0

  // Gravity Feedforward for Arm
  double rawFF = CoralSubsystemConstants.kArmGravityFF * Math.cos(armAngleRadians);
  double gravityFFValue = Math.copySign(Math.max(Math.abs(rawFF), 0.075), rawFF); // Clamp to ±0.05 minimum
  


  // ========================
  // Elevator Slow Zone & FF
  // ========================
  double elevatorPosition = elevatorEncoder.getPosition();
  double distanceToTarget = Math.abs(elevatorCurrentTarget - elevatorPosition);
  double speedFactor = 1.0; // Default full speed

  // Smoothly scale down speed when within slow zone range
  if (distanceToTarget < CoralSubsystemConstants.kElevatorSlowZone) {
      // Linear interpolation: smoothly reduces speed as it nears limit
      speedFactor = CoralSubsystemConstants.kElevatorSlowSpeedFactor + 
          (1 - CoralSubsystemConstants.kElevatorSlowSpeedFactor) * (distanceToTarget / CoralSubsystemConstants.kElevatorSlowZone);
  }

  // Adjusted elevator target based on smooth slow zone factor
  double adjustedElevatorTarget = elevatorPosition + (elevatorCurrentTarget - elevatorPosition) * speedFactor;

  // ========================
  // Arm and Elevator Sequencing
  // ========================
  if (armCurrentTarget > CoralSubsystemConstants.ArmSetpoints.kLevel1) {
      // ------------------
      // If raising (scoring position), move arm first
      // ------------------
      armController.setReference(
          armCurrentTarget,
          ControlType.kMAXMotionPositionControl,
          ClosedLoopSlot.kSlot0,
          gravityFFValue // Arm Gravity Feedforward
      );

      // Once arm is in position, move elevator using adjusted target and elevator FF
      if (Math.abs(armEncoder.getPosition() - armCurrentTarget) < 0.5) {
          elevatorClosedLoopController.setReference(
              adjustedElevatorTarget, // ✅ Slow zone applied target
              ControlType.kMAXMotionPositionControl,
              ClosedLoopSlot.kSlot0,
              CoralSubsystemConstants.kElevatorGravityFF // Elevator Gravity Feedforward
          );
      }

  } else {
      // ------------------
      // If lowering (return to intake/rest), move elevator first
      // ------------------
      elevatorClosedLoopController.setReference(
          adjustedElevatorTarget, // ✅ Slow zone applied target
          ControlType.kMAXMotionPositionControl,
          ClosedLoopSlot.kSlot0,
          CoralSubsystemConstants.kElevatorGravityFF // Elevator Gravity Feedforward
      );

      // Once elevator is mostly down, retract arm with gravity FF
      if (Math.abs(elevatorEncoder.getPosition() - elevatorCurrentTarget) < 2.0) {
          armController.setReference(
              armCurrentTarget,
              ControlType.kMAXMotionPositionControl,
              ClosedLoopSlot.kSlot0,
              gravityFFValue // Arm Gravity Feedforward
          );
      }
  }
}

  /**
   * Drive the arm and elevator motors to their respective setpoints. This will use MAXMotion
   * position control which will allow for a smooth acceleration and deceleration to the mechanisms'
   * setpoints.
   */
//   private void moveToSetpoint() {
//     // Convert encoder position to real-world arm angle in degrees
//     double armAngleDegrees = armEncoder.getPosition() * CoralSubsystemConstants.kArmDegreesPerEncoderTick;
//     double armAngleRadians = Math.toRadians(armAngleDegrees-90); // offset so down is 90 degrees instead of 0
//     double gravityFFValue = CoralSubsystemConstants.kArmGravityFF * Math.cos(armAngleRadians);

//     // Calculate how close we are to limits
//     double elevatorPosition = elevatorEncoder.getPosition();
//     boolean nearTop = (CoralSubsystemConstants.ElevatorSetpoints.kLevel4 - elevatorPosition) < kElevatorSlowZone;
//     boolean nearBottom = elevatorPosition < kElevatorSlowZone;

//     // Calculate speed reduction if near limits
//     double speedFactor = (nearTop || nearBottom) ? kElevatorSlowSpeedFactor : 1.0;

//     // Apply dynamic speed reduction by adjusting target dynamically
//     double adjustedElevatorTarget = elevatorPosition + (elevatorCurrentTarget - elevatorPosition) * speedFactor;

//     // Moving to a scoring position (elevator up, arm out)
//     if (armCurrentTarget > CoralSubsystemConstants.ArmSetpoints.kLevel1) {  
//         // Move arm first with Gravity Feedforward
//         armController.setReference(armCurrentTarget,ControlType.kMAXMotionPositionControl,ClosedLoopSlot.kSlot0,gravityFFValue);

//         // Wait for arm to reach position before moving elevator up
//         if (Math.abs(armEncoder.getPosition() - armCurrentTarget) < 0.5) {
//             elevatorClosedLoopController.setReference(elevatorCurrentTarget,ControlType.kMAXMotionPositionControl);
//         }
//       } 
//     // Returning from a scoring position (elevator down first, then arm)
//     else {
//         // Move elevator down first
//         elevatorClosedLoopController.setReference(adjustedElevatorTarget,ControlType.kMAXMotionPositionControl);

//         // Wait for elevator to lower before moving arm back
//         if (Math.abs(elevatorEncoder.getPosition() - elevatorCurrentTarget) < 2.0) { 
//             armController.setReference(armCurrentTarget,ControlType.kMAXMotionPositionControl,ClosedLoopSlot.kSlot0,gravityFFValue);
//         }
//     }
// }

  /** Zero the elevator encoder when the limit switch is pressed. */
  // private void zeroElevatorOnLimitSwitch() {
  //   if (!wasResetByLimit && !elevatorLimitSwitch.get()) {
  //     // Zero the encoder only when the limit switch is switches from "unpressed" to "pressed" to
  //     // prevent constant zeroing while pressed
  //     elevatorEncoder.setPosition(0);
  //     wasResetByLimit = true;
  //   } else if (elevatorLimitSwitch.get()) {
  //     wasResetByLimit = false;
  //   }
  // }

  /** Zero the arm and elevator encoders when the user button is pressed on the roboRIO. */
  private void zeroOnUserButton() {
    if (!wasResetByButton && RobotController.getUserButton()) {
      // Zero the encoders only when button switches from "unpressed" to "pressed" to prevent
      // constant zeroing while pressed
      wasResetByButton = true;
      armEncoder.setPosition(0);
      elevatorEncoder.setPosition(0);
    } else if (!RobotController.getUserButton()) {
      wasResetByButton = false;
    }
  }
  
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// 
  private boolean hasGamePiece = false;

  /** Set the intake motor power in the range of [-1, 1]. */
  private void setIntakePower(double power) {
    if (hasGamePiece && power == 0.0) {
        intakeMotor.set(CoralSubsystemConstants.IntakeSetpoints.kHold); // Hold only if game piece detected
    } else {
        intakeMotor.set(power);
    }
}


  private void setFrontIntakePower(double power) {
    frontIntakeMotor.set(power);
  }



  /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
   * positions for the given setpoint.
   */
  public Command setSetpointCommand(Setpoint setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case kFeederStation:
              armCurrentTarget = ArmSetpoints.kFeederStation;
              elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;
              break;
            case kLevel1:
              armCurrentTarget = ArmSetpoints.kLevel1;
              elevatorCurrentTarget = ElevatorSetpoints.kLevel1;
              break;
            case kLevel2:
              armCurrentTarget = ArmSetpoints.kLevel2;
              elevatorCurrentTarget = ElevatorSetpoints.kLevel2;
              break;
            case kLevel3:
              armCurrentTarget = ArmSetpoints.kLevel3;
              elevatorCurrentTarget = ElevatorSetpoints.kLevel3;
              break;
            case kLevel4:
              armCurrentTarget = ArmSetpoints.kLevel4;
              elevatorCurrentTarget = ElevatorSetpoints.kLevel4;
              break;
            case kKnockBack: // NEW CASE ADDED
              armCurrentTarget = ArmSetpoints.kKnockBack;
              elevatorCurrentTarget = ElevatorSetpoints.kLevel1;
              break;
            case kAlgaeLow: // NEW CASE ADDED
              armCurrentTarget = ArmSetpoints.kAlgaeLow;
              elevatorCurrentTarget = ElevatorSetpoints.kLevel1;
              break;
            case kAlgaeHigh: // NEW CASE ADDED
              armCurrentTarget = ArmSetpoints.kAlgaeHigh;
              elevatorCurrentTarget = ElevatorSetpoints.kLevelA;
              break;
            case kAlgaeScore1: // NEW CASE ADDED
              armCurrentTarget = ArmSetpoints.kAlgaeScore1;
              elevatorCurrentTarget = ElevatorSetpoints.kLevel4;
              break;
            case kAlgaeScore2: // NEW CASE ADDED
              armCurrentTarget = ArmSetpoints.kAlgaeScore2;
              elevatorCurrentTarget = ElevatorSetpoints.kLevel4;
              break;
          }
        });
  }

  /**
   * Command to run the intake motor. When the command is interrupted, e.g. the button is released,
   * the motor will stop.
   */
  public Command reverseIntakeCommand() {
    return this.startEnd(
        () -> {
            this.setIntakePower(IntakeSetpoints.kReverse);
        },
        () -> {
            this.setIntakePower(0.0);
            ledSubsystem.clearCoralFlash(); // 🔹 Revert LED after ejecting
        }
    ).withTimeout(0.50);
}


  public Command reverseFrontIntakeCommand() {
    return this.startEnd(
        () -> this.setFrontIntakePower(FrontIntakeSetpoints.kReverse), () -> this.setFrontIntakePower(0.0))
        .withTimeout(0.50);
  }

  /**
   * Command to reverses the intake motor. When the command is interrupted, e.g. the button is
   * released, the motor will stop.
   */
  public Command runIntakeCommand() {
    return this.startEnd(
        () -> {
            hasGamePiece = false; // Reset detection at intake start
            setIntakePower(CoralSubsystemConstants.IntakeSetpoints.kForward); // May hold later
            setFrontIntakePower(FrontIntakeSetpoints.kForward); // Just runs
        },
        () -> {
            setIntakePower(0.0); // May apply hold
            setFrontIntakePower(0.0); // Always clean stop
        }
    );
}



  public Command runFrontIntakeCommand() {
    return this.startEnd(
        () -> this.setFrontIntakePower(IntakeSetpoints.kForward), () -> this.setFrontIntakePower(0.0));    
  }
  
  public Command shortRunFrontIntakeCommand() {
    return this.startEnd(
        () -> this.setFrontIntakePower(IntakeSetpoints.kForward), () -> this.setFrontIntakePower(0.0))   
        .withTimeout(0.50); 
  }

  public Command waitUntilIntakeSafe(double armCurrentTarget) {
    return this.run(() -> {})
      .until(() -> Math.abs(armEncoder.getPosition() - armCurrentTarget) <0.5 );
  } 
  public Command waitUntilElevatorInPosition(double elevatorCurrentTarget) {
    return this.run(() -> {})
      .until(() -> Math.abs(elevatorEncoder.getPosition() - elevatorCurrentTarget) <0.5 );
  } 
// Add this to CoralSubsystem.java
public double getArmPosition() {
  return armEncoder.getPosition(); // Returns current arm encoder position
}

// Call this to apply normal config
public void applyNormalArmConfig() {
  armMotor.configure(
      Configs.CoralSubsystem.armConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kNoPersistParameters
  );
}

// Call this to apply fast config
public void applyFastArmConfig() {
  armMotor.configure(
      Configs.CoralSubsystem.armFastConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kNoPersistParameters
  );
}


  @Override
  public void periodic() {
    moveToSetpoint();
    //zeroElevatorOnLimitSwitch(); 
    zeroOnUserButton();

    double current = intakeMotor.getOutputCurrent();
    hasGamePiece = current > 36.0; // Game piece detected?
    SmartDashboard.putNumber("Coral/Intake Current", current);

    if (hasGamePiece) {
      ledSubsystem.flashOnceForCoral(0.93, 2.0); // White
  }
  
  
    // Display subsystem values
    SmartDashboard.putNumber("Arm Target Position", armCurrentTarget);
    SmartDashboard.putNumber("Actual Position", armEncoder.getPosition());
    SmartDashboard.putNumber("Elevator Target Position", elevatorCurrentTarget);
    SmartDashboard.putNumber("Elevator Actual Position", elevatorEncoder.getPosition());
    SmartDashboard.putNumber("Coral Intake Applied Output", intakeMotor.getAppliedOutput());
  
  }
}
