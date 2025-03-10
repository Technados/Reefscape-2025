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
import edu.wpi.first.wpilibj.DigitalInput;

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
    kAlgaeHigh;
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
  private boolean wasResetByLimit = false;
  private double armCurrentTarget = ArmSetpoints.kLevel1;
  private double elevatorCurrentTarget = ElevatorSetpoints.kLevel1;

  // Elevator Limit Switch
  //private DigitalInput elevatorLimitSwitch = new DigitalInput(6);

  // safe elevator slow down zone near limits for dynamic speed reduction approaching limits
  private static final double kElevatorSlowZone = 10.0; //encoder ticks before top/bottom to start slow zone 
  private static final double kElevatorSlowSpeedFactor = 0.4; // 40% of full speed when near limit

  public CoralSubsystem() {
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

  }


  /**
   * Drive the arm and elevator motors to their respective setpoints. This will use MAXMotion
   * position control which will allow for a smooth acceleration and deceleration to the mechanisms'
   * setpoints.
   */
  private void moveToSetpoint() {
    // Convert encoder position to real-world arm angle in degrees
    double armAngleDegrees = armEncoder.getPosition() * CoralSubsystemConstants.kArmDegreesPerEncoderTick;
    double armAngleRadians = Math.toRadians(armAngleDegrees);

    // Calculate how close we are to limits
    double elevatorPosition = elevatorEncoder.getPosition();
    boolean nearTop = (CoralSubsystemConstants.ElevatorSetpoints.kLevel4 - elevatorPosition) < kElevatorSlowZone;
    boolean nearBottom = elevatorPosition < kElevatorSlowZone;

    // Calculate speed reduction if near limits
    double speedFactor = (nearTop || nearBottom) ? kElevatorSlowSpeedFactor : 1.0;

    // Apply dynamic speed reduction by adjusting target dynamically
    double adjustedElevatorTarget = elevatorPosition + (elevatorCurrentTarget - elevatorPosition) * speedFactor;


    // Calculate Gravity Feedforward (GravityFF)
    double gravityFFValue = CoralSubsystemConstants.kGravityFF * Math.cos(armAngleRadians);

    // Moving to a scoring position (elevator up, arm out)
    if (armCurrentTarget > CoralSubsystemConstants.ArmSetpoints.kLevel1) {  
        // Move arm first with Gravity Feedforward
        armController.setReference(armCurrentTarget,ControlType.kMAXMotionPositionControl,ClosedLoopSlot.kSlot0,gravityFFValue);

        // Wait for arm to reach position before moving elevator up
        if (Math.abs(armEncoder.getPosition() - armCurrentTarget) < 0.5) {
            elevatorClosedLoopController.setReference(elevatorCurrentTarget,ControlType.kMAXMotionPositionControl);
        }
      } 
    // Returning from a scoring position (elevator down first, then arm)
    else {
        // Move elevator down first
        elevatorClosedLoopController.setReference(adjustedElevatorTarget,ControlType.kMAXMotionPositionControl);

        // Wait for elevator to lower before moving arm back
        if (Math.abs(elevatorEncoder.getPosition() - elevatorCurrentTarget) < 2.0) { 
            armController.setReference(armCurrentTarget,ControlType.kMAXMotionPositionControl,ClosedLoopSlot.kSlot0,gravityFFValue);
        }
    }
}

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

  /** Set the intake motor power in the range of [-1, 1]. */
  private void setIntakePower(double power) {
    intakeMotor.set(power);
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
          }
        });
  }

  /**
   * Command to run the intake motor. When the command is interrupted, e.g. the button is released,
   * the motor will stop.
   */
  public Command reverseIntakeCommand() {
    return this.startEnd(
        () -> this.setIntakePower(IntakeSetpoints.kReverse), () -> this.setIntakePower(0.0))
        .withTimeout(0.50);
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
            this.setIntakePower(IntakeSetpoints.kForward);
            this.setFrontIntakePower(FrontIntakeSetpoints.kForward);
        }, 
        () -> {
            this.setIntakePower(0.0);
            this.setFrontIntakePower(0.0);
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

  @Override
  public void periodic() {
    moveToSetpoint();
    //zeroElevatorOnLimitSwitch(); 
    zeroOnUserButton();

  
    // Display subsystem values
    SmartDashboard.putNumber("Arm Target Position", armCurrentTarget);
    SmartDashboard.putNumber("Actual Position", armEncoder.getPosition());
    SmartDashboard.putNumber("Elevator Target Position", elevatorCurrentTarget);
    SmartDashboard.putNumber("Elevator Actual Position", elevatorEncoder.getPosition());
    SmartDashboard.putNumber("Coral Intake Applied Output", intakeMotor.getAppliedOutput());
  
  }
}
