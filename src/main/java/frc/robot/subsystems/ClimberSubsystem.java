// package frc.robot.subsystems;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Configs;
// import frc.robot.Constants.ClimberSubsystemConstants;
// import frc.robot.Constants.ClimberSubsystemConstants.ArmSetpoints;
// import frc.robot.Constants.ClimberSubsystemConstants.ClimbSetpoints;


// public class ClimberSubsystem extends SubsystemBase {
//   /** Subsystem-wide setpoints */
//   public enum Setpoint {
//     kFeederStation,
//     kLevel1,
//     kLevel2,
//     kLevel3,
//     kLevel4;
//   }

//   // Initialize arm SPARK. We will use MAXMotion position control for the arm, so we also need to
//   // initialize the closed loop controller and encoder.
//   private SparkMax climbArmMotor =
//       new SparkMax(ClimberSubsystemConstants.kClimbArmMotorCanId, MotorType.kBrushless);
//   private SparkClosedLoopController armController = climbArmMotor.getClosedLoopController();
//   private RelativeEncoder climbArmEncoder = climbArmMotor.getEncoder();

//   // Initialize intake SPARK. We will use open loop control for this so we don't need a closed loop
//   // controller like above.
//   private SparkMax climbMotor = new SparkMax(ClimberSubsystemConstants.kClimbMotorCanId, MotorType.kBrushless);
  
//   // Member variables for subsystem state management
//   private boolean wasResetByButton = false;
//   private double armCurrentTarget = ArmSetpoints.kStow;

//   public ClimberSubsystem() {
//     /*
//      * Apply the appropriate configurations to the SPARKs.
//      *
//      * kResetSafeParameters is used to get the SPARK to a known state. This
//      * is useful in case the SPARK is replaced.
//      *
//      * kPersistParameters is used to ensure the configuration is not lost when
//      * the SPARK loses power. This is useful for power cycles that may occur
//      * mid-operation.
//      */
//     climbArmMotor.configure(
//         Configs.ClimberSubsystem.climbArmConfig,
//         ResetMode.kResetSafeParameters,
//         PersistMode.kPersistParameters);
//     climbMotor.configure(
//         Configs.ClimberSubsystem.climbConfig,
//         ResetMode.kResetSafeParameters,
//         PersistMode.kPersistParameters);


//     // Zero arm and elevator encoders on initialization
//     climbArmEncoder.setPosition(0);
//   }


//   /**
//    * Drive the arm and elevator motors to their respective setpoints. This will use MAXMotion
//    * position control which will allow for a smooth acceleration and deceleration to the mechanisms'
//    * setpoints.
//    */
//   private void moveToSetpoint() {
//     armController.setReference(armCurrentTarget, ControlType.kMAXMotionPositionControl);
//   }

//   /** Zero the arm and elevator encoders when the user button is pressed on the roboRIO. */
//   private void zeroOnUserButton() {
//     if (!wasResetByButton && RobotController.getUserButton()) {
//       // Zero the encoders only when button switches from "unpressed" to "pressed" to prevent
//       // constant zeroing while pressed
//       wasResetByButton = true;
//       climbArmEncoder.setPosition(0);
//     } else if (!RobotController.getUserButton()) {
//       wasResetByButton = false;
//     }
//   }

//   /** Set the intake motor power in the range of [-1, 1]. */
//   private void setClimbPower(double power) {
//     climbMotor.set(power);
//   }
  

//   /**
//    * Command to run the intake motor. When the command is interrupted, e.g. the button is released,
//    * the motor will stop.
//    */
//   public Command runClimbCommand() {
//     return this.startEnd(
//         () -> this.setClimbPower(ClimbSetpoints.kForward), () -> this.setClimbPower(0.0));
//   }

//   /**
//    * Command to reverses the intake motor. When the command is interrupted, e.g. the button is
//    * released, the motor will stop.
//    */
//   public Command reverseClimbCommand() {
//     return this.startEnd(
//         () -> this.setClimbPower(ClimbSetpoints.kReverse), () -> this.setClimbPower(0.0));
//   }

//   @Override
//   public void periodic() {
//     moveToSetpoint();
//     zeroOnUserButton();

//   }
// }
