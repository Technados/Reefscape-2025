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
import frc.robot.Constants.CoralSubsystemConstants.IntakeSetpoints;


public class AlgaeSubsystem extends SubsystemBase {

  private boolean hasGamePiece = false; // Tracks if a game piece is detected
  
  // Initialize intake SPARK. We will use open loop control for this so we don't need a closed loop
  // controller like above.
  private SparkMax intakeMotor =
      new SparkMax(AlgaeSubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);

  // Member variables for subsystem state management
  // private boolean stowWhenIdle = true;

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
  private void setIntakePower(double power) {
    if (hasGamePiece && power == 0.0) {
        intakeMotor.set(AlgaeSubsystemConstants.IntakeSetpoints.kHold); // Apply hold power
    } else {
        intakeMotor.set(power);
    }
}

    /**
     * Command to run the algae intake forward.
     */
    public Command runIntakeCommand() {
      return this.startEnd(
          () -> {
              hasGamePiece = false; // Reset when running intake
              setIntakePower(AlgaeSubsystemConstants.IntakeSetpoints.kForward);
          },
          () -> {
              hasGamePiece = true; // Assume a piece is held after stopping
              setIntakePower(0.0);
          }
      );
  }

  /**
   * Command to reverse the algae intake.
   */
  public Command reverseIntakeCommand() {
      return this.startEnd(
          () -> {
              hasGamePiece = false; // Releasing the game piece
              setIntakePower(AlgaeSubsystemConstants.IntakeSetpoints.kReverse);
          },
          () -> {
              hasGamePiece = false; // Ensure holding power does not apply after ejecting
              setIntakePower(0.0);
          }
      );
  }
  
  @Override
  public void periodic() {
      double current = intakeMotor.getOutputCurrent();
      hasGamePiece = (current > 10.0); // Adjust threshold as needed
      SmartDashboard.putNumber("Algae/Intake Current", current);
  }
}
