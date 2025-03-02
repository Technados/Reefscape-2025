// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.AlgaeSubsystemConstants;


public class AlgaeSubsystem extends SubsystemBase {

  // Initialize intake SPARK. We will use open loop control for this so we don't need a closed loop
  // controller like above.
  private SparkMax intakeMotor =
      new SparkMax(AlgaeSubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);

  // Member variables for subsystem state management
  private boolean stowWhenIdle = true;

  public AlgaeSubsystem() {
    /*
     * Apply the configuration to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK to a known state. This
     * is useful in case the SPARK is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    intakeMotor.configure(
        Configs.AlgaeSubsystem.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

  }

  /**
   * Command to run the algae intake. This will extend the arm to its "down" position and run the
   * motor at its "forward" power to intake the ball.
   *
   * <p>This will also update the idle state to hold onto the ball when this command is not running.
   */
  public Command runIntakeCommand() {
    return this.run(
        () -> {
          stowWhenIdle = false;
          setIntakePower(AlgaeSubsystemConstants.IntakeSetpoints.kForward);
          //setIntakePosition(AlgaeSubsystemConstants.ArmSetpoints.kDown);
        });
  }

  /**
   * Command to run the algae intake in reverse.
   *
   * 
   */
  public Command reverseIntakeCommand() {
    return this.run(
        () -> {
          stowWhenIdle = true;
          setIntakePower(AlgaeSubsystemConstants.IntakeSetpoints.kReverse);
        });
  }

  /** Command to force the subsystem into its "stow" state. */
  public Command stowCommand() {
    return this.runOnce(
        () -> {
          stowWhenIdle = true;
        });
  }

  /**
   * Command to run when the intake is not actively running. When in the "hold" state, the intake
   * will stay in the "hold" position and run the motor at its "hold" power to hold onto the ball.
   * When in the "stow" state, the intake will stow the arm in the "stow" position and stop the
   * motor.
   */
  public Command idleCommand() {
    return this.run(
        () -> {
          if (stowWhenIdle) {
            setIntakePower(0.0);
          } else {
            setIntakePower(AlgaeSubsystemConstants.IntakeSetpoints.kHold);
          }
        });
  }

  /** Set the intake motor power in the range of [-1, 1]. */
  private void setIntakePower(double power) {
    intakeMotor.set(power);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Algae/Intake/Applied Output", intakeMotor.getAppliedOutput());
  }
}
