// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CoralSubsystemConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.AlgaeSubsystemConstants.ArmSetpoints;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.CoralSubsystem.Setpoint;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.Alignment;
import frc.robot.subsystems.ClimberSubsystem;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.pathplanner.lib.util.GeometryUtil;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final CoralSubsystem m_coralSubSystem = new CoralSubsystem();
  private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
  // private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

  //private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  //Create Commands


  // create autoChooser
  private final SendableChooser<String> autoChooser = new SendableChooser<>();

  // Commands for PathPlanner
  private void registerPathPlannerCommands() {
    NamedCommands.registerCommand("CoralFeed", m_coralSubSystem.setSetpointCommand(Setpoint.kFeederStation));
    //NamedCommands.registerCommand("CoralPositionL2", m_coralSubSystem.setSetpointCommand(Setpoint.kLevel2));
    //NamedCommands.registerCommand("CoralPositionL3", m_coralSubSystem.setSetpointCommand(Setpoint.kLevel3));
    //NamedCommands.registerCommand("CoralPositionL4", m_coralSubSystem.setSetpointCommand(Setpoint.kLevel4));
    //NamedCommands.registerCommand("KnockBack", m_coralSubSystem.setSetpointCommand(Setpoint.kKnockBack));
    NamedCommands.registerCommand("RunIntake", m_coralSubSystem.runIntakeCommand().withTimeout(1.0));
    //NamedCommands.registerCommand("ScoreCoral", m_coralSubSystem.reverseIntakeCommand());

    NamedCommands.registerCommand("ScoreL2",
    new SequentialCommandGroup(

        m_coralSubSystem.waitUntilElevatorInPosition(CoralSubsystemConstants.ElevatorSetpoints.kLevel2),
        m_coralSubSystem.reverseIntakeCommand(),
        m_coralSubSystem.setSetpointCommand(Setpoint.kKnockBack),
        m_coralSubSystem.waitUntilIntakeSafe(CoralSubsystemConstants.ArmSetpoints.kKnockBack)
        )
    );

    NamedCommands.registerCommand("MoveToL2",
    new SequentialCommandGroup(
        m_coralSubSystem.setSetpointCommand(Setpoint.kLevel2),
        m_coralSubSystem.waitUntilIntakeSafe(CoralSubsystemConstants.ArmSetpoints.kLevel2)
    ));

    NamedCommands.registerCommand("ScoreL3",
    new SequentialCommandGroup(

        m_coralSubSystem.waitUntilElevatorInPosition(CoralSubsystemConstants.ElevatorSetpoints.kLevel3),
        m_coralSubSystem.reverseIntakeCommand(),
        m_coralSubSystem.setSetpointCommand(Setpoint.kKnockBack),
        m_coralSubSystem.waitUntilIntakeSafe(CoralSubsystemConstants.ArmSetpoints.kKnockBack)
        )
    );

    NamedCommands.registerCommand("MoveToL3",
    new SequentialCommandGroup(
        m_coralSubSystem.setSetpointCommand(Setpoint.kLevel3),
        m_coralSubSystem.waitUntilIntakeSafe(CoralSubsystemConstants.ArmSetpoints.kLevel3)
    ));

    NamedCommands.registerCommand("ScoreL4",
    new SequentialCommandGroup(

        m_coralSubSystem.waitUntilElevatorInPosition(CoralSubsystemConstants.ElevatorSetpoints.kLevel4),
        m_coralSubSystem.reverseIntakeCommand(),
        m_coralSubSystem.setSetpointCommand(Setpoint.kKnockBack),
        m_coralSubSystem.waitUntilIntakeSafe(CoralSubsystemConstants.ArmSetpoints.kKnockBack)
        )
    );

    NamedCommands.registerCommand("MoveToL4",
    new SequentialCommandGroup(
        m_coralSubSystem.setSetpointCommand(Setpoint.kLevel4),
        m_coralSubSystem.waitUntilIntakeSafe(CoralSubsystemConstants.ArmSetpoints.kLevel3)
    ));

}
////////////////////////////////////////////////////////////////////////////////////////////////////////


  // The driver's controller
  CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  // The operator's controller
  CommandXboxController m_operatorController =
      new CommandXboxController(OIConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // call the pathplanner command reg
    registerPathPlannerCommands();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true),
            m_robotDrive));

    // Set the ball intake to in/out when not running based on internal state
    m_algaeSubsystem.setDefaultCommand(m_algaeSubsystem.idleCommand());

    // register auto options to the shuffleboard           
    autoChooser.addOption("none", null);
    autoChooser.addOption("1Meter", "1Meter");
    autoChooser.addOption("180", "180");
    autoChooser.addOption("Test Path", "Test Path");
    autoChooser.addOption("BMDR3", "BMDR3");
    autoChooser.addOption("BackMiddle", "BackMiddle");
    

    // Creating a new shuffleboard tab and adding the autoChooser
    Shuffleboard.getTab("PathPlanner Autonomous").add(autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    // Driver Controller
    // Left Stick Button -> Set swerve to X
    m_driverController.leftStick().whileTrue(m_robotDrive.setXCommand());
    
    // Hole left bumper to run intake and align to left/right substation
    m_driverController.leftBumper().whileTrue(m_coralSubSystem.runIntakeCommand());
    //.alongWith(m_robotDrive.alignToSubstationCommand()));

    // Start Button -> Zero swerve heading
    m_driverController.start().onTrue(m_robotDrive.zeroHeadingCommand());

    // Right Bumper -> Enable Slow Mode While Held
    m_driverController.rightBumper().whileTrue(
    new RunCommand(() -> m_robotDrive.setSlowMode(true), m_robotDrive)
    ).onFalse(new InstantCommand(() -> m_robotDrive.setSlowMode(false)));
    

    // Operator Controller
    // Reef Scoring Alignments - d-pad aligns left, right, or center to reef sides using apriltag
    m_operatorController.povLeft().whileTrue(m_robotDrive.alignToReefCommand(Alignment.LEFT));
    //.onFalse(new InstantCommand(() -> m_robotDrive.stopMovement())); // Stops on release
    m_operatorController.povRight().whileTrue(m_robotDrive.alignToReefCommand(Alignment.RIGHT));
    //.onFalse(new InstantCommand(() -> m_robotDrive.stopMovement())); // Stops on release
    m_operatorController.povUp().whileTrue(m_robotDrive.alignToReefCommand(Alignment.CENTER));
    //.onFalse(new InstantCommand(() -> m_robotDrive.stopMovement())); // Stops on release

    // Left Bumper -> Run coral intake
    // Right Bumper -> Run coral intake in reverse
   // m_operatorController.rightBumper().whileTrue(m_coralSubSystem.reverseIntakeCommand());
    // B Button -> Elevator/Arm to human player position, set ball intake to stow
    // when idle
    m_operatorController.b().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kFeederStation).alongWith(m_algaeSubsystem.stowCommand()));
    
    // START Button -> Elevator/Arm to level 1 position
    m_operatorController.start().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel1));
    
    // A Button -> Elevator/Arm to level 2 position
    m_operatorController.a().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel2));

    // X Button -> Elevator/Arm to level 3 position
    m_operatorController.x().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel3));

    // Y Button -> Elevator/Arm to level 4 position
    m_operatorController.y().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel4));

    /**
     * Right Trigger -> Run Coral Scoring Sequence
     * Reverse intake runs for fixed duration (set in subsytem)
     * Intake arm rotates back to safe position at same time intake runs reverse
     * AFTER intake arm is at safe position, elevator returns to zero
     */
    m_operatorController
    .rightTrigger(OIConstants.kTriggerButtonThreshold).onTrue(
        m_coralSubSystem.reverseIntakeCommand()
        .andThen(() -> m_coralSubSystem.setSetpointCommand(Setpoint.kKnockBack).schedule()) // Move intake to KnockBack
        //.andThen(m_coralSubSystem.waitUntilIntakeSafe(CoralSubsystemConstants.ArmSetpoints.kKnockBack)) // Wait for safe position
        .andThen(() -> m_coralSubSystem.setSetpointCommand(Setpoint.kLevel1).schedule()) // Lower Elevator first
    );


    // Right Trigger -> Run ball intake, set to leave out when idle
   // m_operatorController
   //     .rightTrigger(OIConstants.kTriggerButtonThreshold)
   //     .whileTrue(m_algaeSubsystem.runIntakeCommand());

    // Left Trigger -> Run ball intake in reverse, set to stow when idle
    //m_operatorController
    //   .leftTrigger(OIConstants.kTriggerButtonThreshold)
  // .whileTrue(m_algaeSubsystem.reverseIntakeCommand());

  }

  public double getSimulationTotalCurrentDraw() {
    // for each subsystem with simulation
    return m_coralSubSystem.getSimulationCurrentDraw()
        + m_algaeSubsystem.getSimulationCurrentDraw();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

   public Command getAutonomousCommand() {
    // Check if a path is selected
    
    if (autoChooser.getSelected() == null) {
        return null;
    }

    String selectedPath = autoChooser.getSelected();

    // Build and return the selected autonomous command
    return AutoBuilder.buildAuto(selectedPath);
}

}
