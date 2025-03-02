package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CoralSubsystemConstants;
import frc.robot.Constants.CoralSubsystemConstants.ArmSetpoints;
import frc.robot.Constants.CoralSubsystemConstants.ElevatorSetpoints;
import frc.robot.Constants.CoralSubsystemConstants.IntakeSetpoints;
import frc.robot.Constants.SimulationRobotConstants;
import edu.wpi.first.wpilibj.DigitalInput;

public class CoralSubsystem extends SubsystemBase {
  /** Subsystem-wide setpoints */
  public enum Setpoint {
    kFeederStation,
    kLevel1,
    kLevel2,
    kLevel3,
    kLevel4,
    kKnockBack;
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

  // Member variables for subsystem state management
  private boolean wasResetByButton = false;
  private boolean wasResetByLimit = false;
  private double armCurrentTarget = ArmSetpoints.kLevel1;
  private double elevatorCurrentTarget = ElevatorSetpoints.kLevel1;

  // Simulation setup and variables
  private DigitalInput elevatorLimitSwitch = new DigitalInput(0);
  private DCMotor elevatorMotorModel = DCMotor.getNEO(1);
  private SparkMaxSim elevatorMotorSim;
  private SparkLimitSwitchSim elevatorLimitSwitchSim;
  private final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          elevatorMotorModel,
          SimulationRobotConstants.kElevatorGearing,
          SimulationRobotConstants.kCarriageMass,
          SimulationRobotConstants.kElevatorDrumRadius,
          SimulationRobotConstants.kMinElevatorHeightMeters,
          SimulationRobotConstants.kMaxElevatorHeightMeters,
          true,
          SimulationRobotConstants.kMinElevatorHeightMeters,
          0.0,
          0.0);

  private DCMotor armMotorModel = DCMotor.getNEO(1);
  private SparkMaxSim armMotorSim;
  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          armMotorModel,
          SimulationRobotConstants.kArmReduction,
          SingleJointedArmSim.estimateMOI(
              SimulationRobotConstants.kArmLength, SimulationRobotConstants.kArmMass),
          SimulationRobotConstants.kArmLength,
          SimulationRobotConstants.kMinAngleRads,
          SimulationRobotConstants.kMaxAngleRads,
          true,
          SimulationRobotConstants.kMinAngleRads,
          0.0,
          0.0);

  // Mechanism2d setup for subsystem
  private final Mechanism2d m_mech2d = new Mechanism2d(50, 50);
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("ElevatorArm Root", 25, 0);
  private final MechanismLigament2d m_elevatorMech2d =
      m_mech2dRoot.append(
          new MechanismLigament2d(
              "Elevator",
              SimulationRobotConstants.kMinElevatorHeightMeters
                  * SimulationRobotConstants.kPixelsPerMeter,
              90));
  private final MechanismLigament2d m_armMech2d =
      m_elevatorMech2d.append(
          new MechanismLigament2d(
              "Arm",
              SimulationRobotConstants.kArmLength * SimulationRobotConstants.kPixelsPerMeter,
              180 - Units.radiansToDegrees(SimulationRobotConstants.kMinAngleRads) - 90));


   // Display subsystem values
    // SmartDashboard.putNumber("SmartDashboard/Coral Subsystem/Coral/Arm/Target Position", armCurrentTarget);
    // SmartDashboard.putNumber("SmartDashboard/Coral Subsystem/Coral/Arm/Actual Position", armEncoder.getPosition());
    // SmartDashboard.putNumber("SmartDashboard/Coral Subsystem/Coral/Elevator/Target Position", elevatorCurrentTarget);
    // SmartDashboard.putNumber("SmartDashboard/Coral Subsystem/Coral/Elevator/Actual Position", elevatorEncoder.getPosition());
    // SmartDashboard.putNumber("SmartDashboard/Coral Subsystem/Coral/Intake/Applied Output", intakeMotor.getAppliedOutput());
    // SmartDashboard.putBoolean("SmartDashboard/Coral Subsystem/Coral/Intake/Limit Switch", !elevatorLimitSwitch.get());



  // Coral Tab / Entries
  private ShuffleboardTab coralTab = Shuffleboard.getTab("Coral Subsystem");
  private GenericEntry armTargetPosEntry = coralTab.add("Arm Target Position", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
  private GenericEntry armActualPosEntry = coralTab.add("Arm Actual Position", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
  private GenericEntry elevatorTargetPosEntry = coralTab.add("Elevator Target Position", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
  private GenericEntry elevatorActualPosEntry = coralTab.add("Elevator Actual Position", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
  private GenericEntry intakeAppliedOutputEntry = coralTab.add("Intake Applied Output", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
  private GenericEntry intakeLimitSwitchEntry = coralTab.add("Intake Limit Switch", 0).withWidget(BuiltInWidgets.kBooleanBox).getEntry();





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

    // Display mechanism2d
    //Shuffleboard.getTab("Coral Subsystem").add(m_mech2d);
    SmartDashboard.putData("SmartDashboard/Coral Subsystem", m_mech2d);

    // Zero arm and elevator encoders on initialization
    armEncoder.setPosition(0);
    elevatorEncoder.setPosition(0);

    // Initialize simulation values
    elevatorMotorSim = new SparkMaxSim(elevatorMotor, elevatorMotorModel);
    elevatorLimitSwitchSim = new SparkLimitSwitchSim(elevatorMotor, false);
    armMotorSim = new SparkMaxSim(armMotor, armMotorModel);
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
        elevatorClosedLoopController.setReference(elevatorCurrentTarget,ControlType.kMAXMotionPositionControl);

        // Wait for elevator to lower before moving arm back
        if (Math.abs(elevatorEncoder.getPosition() - elevatorCurrentTarget) < 2.0) { 
            armController.setReference(armCurrentTarget,ControlType.kMAXMotionPositionControl,ClosedLoopSlot.kSlot0,gravityFFValue);
        }
    }
}

  /** Zero the elevator encoder when the limit switch is pressed. */
  private void zeroElevatorOnLimitSwitch() {
    if (!wasResetByLimit && !elevatorLimitSwitch.get()) {
      // Zero the encoder only when the limit switch is switches from "unpressed" to "pressed" to
      // prevent constant zeroing while pressed
      elevatorEncoder.setPosition(0);
      wasResetByLimit = true;
    } else if (elevatorLimitSwitch.get()) {
      wasResetByLimit = false;
    }
  }

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

  /**
   * Command to reverses the intake motor. When the command is interrupted, e.g. the button is
   * released, the motor will stop.
   */
  public Command runIntakeCommand() {
    return this.startEnd(
        () -> this.setIntakePower(IntakeSetpoints.kForward), () -> this.setIntakePower(0.0));    
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
    zeroElevatorOnLimitSwitch();
    zeroOnUserButton();

    


    // Display subsystem values
    // SmartDashboard.putNumber("SmartDashboard/Coral Subsystem/Coral/Arm/Target Position", armCurrentTarget);
    // SmartDashboard.putNumber("SmartDashboard/Coral Subsystem/Coral/Arm/Actual Position", armEncoder.getPosition());
    // SmartDashboard.putNumber("SmartDashboard/Coral Subsystem/Coral/Elevator/Target Position", elevatorCurrentTarget);
    // SmartDashboard.putNumber("SmartDashboard/Coral Subsystem/Coral/Elevator/Actual Position", elevatorEncoder.getPosition());
    // SmartDashboard.putNumber("SmartDashboard/Coral Subsystem/Coral/Intake/Applied Output", intakeMotor.getAppliedOutput());
    // SmartDashboard.putBoolean("SmartDashboard/Coral Subsystem/Coral/Intake/Limit Switch", !elevatorLimitSwitch.get());


    // Display subsystem values
    armTargetPosEntry.setDouble(armCurrentTarget);
    armActualPosEntry.setDouble(armEncoder.getPosition());
    elevatorTargetPosEntry.setDouble(elevatorCurrentTarget);
    elevatorActualPosEntry.setDouble(elevatorEncoder.getPosition());
    intakeAppliedOutputEntry.setDouble(intakeMotor.getAppliedOutput());
    intakeLimitSwitchEntry.setBoolean(!elevatorLimitSwitch.get());
    
    



    // Update mechanism2d
    m_elevatorMech2d.setLength(
        SimulationRobotConstants.kPixelsPerMeter * SimulationRobotConstants.kMinElevatorHeightMeters
            + SimulationRobotConstants.kPixelsPerMeter
                * (elevatorEncoder.getPosition() / SimulationRobotConstants.kElevatorGearing)
                * (SimulationRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI));
    m_armMech2d.setAngle(
        180
            - ( // mirror the angles so they display in the correct direction
            Units.radiansToDegrees(SimulationRobotConstants.kMinAngleRads)
                + Units.rotationsToDegrees(
                    armEncoder.getPosition() / SimulationRobotConstants.kArmReduction))
            - 90 // subtract 90 degrees to account for the elevator
        );
  }

  /** Get the current drawn by each simulation physics model */
  public double getSimulationCurrentDraw() {
    return m_elevatorSim.getCurrentDrawAmps() + m_armSim.getCurrentDrawAmps();
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_elevatorSim.setInput(elevatorMotor.getAppliedOutput() * RobotController.getBatteryVoltage());
    m_armSim.setInput(armMotor.getAppliedOutput() * RobotController.getBatteryVoltage());

    // Update sim limit switch
    elevatorLimitSwitchSim.setPressed(m_elevatorSim.getPositionMeters() == 0);

    // Next, we update it. The standard loop time is 20ms.
    m_elevatorSim.update(0.020);
    m_armSim.update(0.020);

    // Iterate the elevator and arm SPARK simulations
    elevatorMotorSim.iterate(
        ((m_elevatorSim.getVelocityMetersPerSecond()
                    / (SimulationRobotConstants.kElevatorDrumRadius * 2.0 * Math.PI))
                * SimulationRobotConstants.kElevatorGearing)
            * 60.0,
        RobotController.getBatteryVoltage(),
        0.02);
    armMotorSim.iterate(
        Units.radiansPerSecondToRotationsPerMinute(
            m_armSim.getVelocityRadPerSec() * SimulationRobotConstants.kArmReduction),
        RobotController.getBatteryVoltage(),
        0.02);

    // SimBattery is updated in Robot.java
  }
}
