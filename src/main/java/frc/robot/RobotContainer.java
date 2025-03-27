// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.CoralSubsystemConstants;
import frc.robot.Constants.OIConstants;
//import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.CoralSubsystem.Setpoint;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.Alignment;
import frc.robot.subsystems.LEDSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems

    // First create subsytems in container
    private final LEDSubsystem m_ledSubsystem = new LEDSubsystem(0); // PWM port 0
    
    private final CoralSubsystem m_coralSubSystem = new CoralSubsystem(m_ledSubsystem);
    private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem(m_ledSubsystem);
    private final DriveSubsystem m_robotDrive = new DriveSubsystem(m_coralSubSystem, m_ledSubsystem);
    
    



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // create autoChooser
  private final SendableChooser<String> autoChooser = new SendableChooser<>();

  // Commands for PathPlanner
  private void registerPathPlannerCommands() {
    NamedCommands.registerCommand("CoralFeed", m_coralSubSystem.setSetpointCommand(Setpoint.kFeederStation));
    //NamedCommands.registerCommand("CoralPositionL2", m_coralSubSystem.setSetpointCommand(Setpoint.kLevel2));
    //NamedCommands.registerCommand("CoralPositionL3", m_coralSubSystem.setSetpointCommand(Setpoint.kLevel3));
    //NamedCommands.registerCommand("CoralPositionL4", m_coralSubSystem.setSetpointCommand(Setpoint.kLevel4));
    NamedCommands.registerCommand("Start", m_coralSubSystem.setSetpointCommand(Setpoint.kLevel1));
    NamedCommands.registerCommand("Intake", m_coralSubSystem.runIntakeCommand().withTimeout(1.5));
    //NamedCommands.registerCommand("Outake", m_coralSubSystem.reverseIntakeCommand());
    NamedCommands.registerCommand("MoveToAlgaeScore", m_coralSubSystem.setSetpointCommand(Setpoint.kAlgaeScore1));
    NamedCommands.registerCommand("MoveToAlgaeLow", m_coralSubSystem.setSetpointCommand(Setpoint.kAlgaeLow));
    NamedCommands.registerCommand("MoveToAlgaeHigh", m_coralSubSystem.setSetpointCommand(Setpoint.kAlgaeHigh));



    // NamedCommands.registerCommand("Intake",
    // new ParallelCommandGroup(

    //     //m_coralSubSystem.waitUntilElevatorInPosition(CoralSubsystemConstants.ElevatorSetpoints.kFeederStation),
    //     m_coralSubSystem.runFrontIntakeCommand(),
    //     m_coralSubSystem.runIntakeCommand()
    //     )
    // );

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
        m_coralSubSystem.setSetpointCommand(Setpoint.kFeederStation)
        //m_coralSubSystem.waitUntilIntakeSafe(CoralSubsystemConstants.ArmSetpoints.kFeederStation)
        )
    );

    NamedCommands.registerCommand("MoveToL4",
    new SequentialCommandGroup(
        m_coralSubSystem.setSetpointCommand(Setpoint.kLevel4),
        m_coralSubSystem.waitUntilIntakeSafe(CoralSubsystemConstants.ArmSetpoints.kLevel3)
    ));

    NamedCommands.registerCommand("ScoreAlgae", 
    new SequentialCommandGroup(
        new InstantCommand(() -> m_coralSubSystem.applyFastArmConfig()),
        m_coralSubSystem.setSetpointCommand(Setpoint.kAlgaeScore2),
        new WaitUntilCommand(() -> Math.abs(m_coralSubSystem.getArmPosition() - 24) < 0.25),
        m_algaeSubsystem.reverseIntakeCommand().withTimeout(0.3),
        m_coralSubSystem.setSetpointCommand(Setpoint.kLevel1),
        new InstantCommand(() -> m_coralSubSystem.applyNormalArmConfig())
    ));

    NamedCommands.registerCommand("AlgaeLow", 
    new SequentialCommandGroup(
        m_coralSubSystem.setSetpointCommand(Setpoint.kAlgaeLow),
        m_coralSubSystem.waitUntilIntakeSafe(Constants.CoralSubsystemConstants.ArmSetpoints.kAlgaeLow),
        m_algaeSubsystem.runIntakeCommand().withTimeout(1.25),
        m_coralSubSystem.setSetpointCommand(Setpoint.kLevel1)
    ));

    NamedCommands.registerCommand("AlgaeHigh", 
    new SequentialCommandGroup(
        m_coralSubSystem.setSetpointCommand(Setpoint.kAlgaeHigh),
        m_coralSubSystem.waitUntilIntakeSafe(Constants.CoralSubsystemConstants.ArmSetpoints.kAlgaeHigh),
        m_algaeSubsystem.runIntakeCommand().withTimeout(1.25),
        m_coralSubSystem.setSetpointCommand(Setpoint.kLevel1)
));

}

////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////

  // The driver's controller
  CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  // The operator's controller
  CommandXboxController m_operatorController =
      new CommandXboxController(OIConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_robotDrive.resetGyroToFieldBackwards();
    // Configure the button bindings
    configureButtonBindings();

    // call the pathplanner command reg
    registerPathPlannerCommands();

    // Configure default commands
m_robotDrive.setDefaultCommand(
    new RunCommand(
        () -> {
            boolean manualSlowMode = m_driverController.rightBumper().getAsBoolean();
            m_robotDrive.updateDriveSlowMode(manualSlowMode); // Auto/Manual slow mode

            m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true
            );
        }, m_robotDrive
    )
);

    // Set the ball intake to in/out when not running based on internal state
    // m_algaeSubsystem.setDefaultCommand(m_algaeSubsystem.idleCommand());

    // register auto options to the shuffleboard           
    autoChooser.addOption("LE", "LE");
    autoChooser.addOption("LF", "LF");
    autoChooser.addOption("RC", "RC");
    autoChooser.addOption("RB", "RB");
    autoChooser.addOption("MDA", "MDA");
    autoChooser.addOption("MDC", "MDC");

    // Creating a new shuffleboard tab and adding the autoChooser
    //Shuffleboard.getTab("PathPlanner Autonomous").add(BlueautoChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    //Shuffleboard.getTab("PathPlanner Autonomous").add(RedautoChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    Shuffleboard.getTab("PathPlanner Autonomous").add(autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  private void configureButtonBindings() {

    // Driver Controller
    // Left Stick Button -> Set swerve to X
    m_driverController.leftStick().whileTrue(m_robotDrive.setXCommand());
    
    // Hole left bumper to run intake and align to left/right substation
    m_driverController.leftBumper().whileTrue(m_coralSubSystem.runIntakeCommand());

    // Run Algae motor w/ 'LT' and reverse with 'RT' button (for now)
    m_driverController.leftTrigger(OIConstants.kTriggerButtonThreshold).onTrue(m_algaeSubsystem.runIntakeCommand());

    // DRIVER A button -> move to algae scoring position
    m_driverController.a().onTrue(
    m_coralSubSystem.setSetpointCommand(Setpoint.kAlgaeScore1)); // Arm 24, Elevator 148 (already in Setpoint enum as Level4)

    m_driverController.rightTrigger(OIConstants.kTriggerButtonThreshold).onTrue(
        new SequentialCommandGroup(
            // STEP 0: Apply fast motion config
            new InstantCommand(() -> m_coralSubSystem.applyFastArmConfig()),
    
            // STEP 1: Move arm to AlgaeScore2 (arm 12, elevator holds)
            m_coralSubSystem.setSetpointCommand(Setpoint.kAlgaeScore2),
    
            // STEP 2: Wait until arm near -23.5, then toss algae
            new WaitUntilCommand(() -> Math.abs(m_coralSubSystem.getArmPosition() - 24) < 0.25),
            m_algaeSubsystem.reverseIntakeCommand().withTimeout(0.3), // Toss algae
    
            // STEP 3: Return to Level 1
            m_coralSubSystem.setSetpointCommand(Setpoint.kLevel1),
    
            // STEP 4: Restore normal config for other movements
            new InstantCommand(() -> m_coralSubSystem.applyNormalArmConfig())
        )
    
        .onlyIf(() -> {
            double elevatorPos = m_coralSubSystem.getElevatorHeight();
            double armPos = m_coralSubSystem.getArmPosition();
    
            boolean armReady = Math.abs(armPos - Constants.CoralSubsystemConstants.ArmSetpoints.kAlgaeScore1) < 1.0;
            boolean elevatorReady = Math.abs(elevatorPos - Constants.CoralSubsystemConstants.ElevatorSetpoints.kLevel4) < 5.0;
    
            return armReady && elevatorReady;
        })

    );

    m_driverController.back().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel1));
    
    

    


    // Driver press 'x' to move arm and elevator to low algae removal position
    m_driverController.x().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kAlgaeLow));
    // Driver press 'y' to move arm and elevator to high algae removal position
    m_driverController.y().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kAlgaeHigh));
  

    // Start Button -> Zero swerve heading
    m_driverController.start().onTrue(m_robotDrive.zeroHeadingCommand());

    // Down on d-pad will reverse front intake only while pressed (if coral stuck)
    m_driverController.povDown().whileTrue(m_coralSubSystem.reverseFrontIntakeCommand());

    // Right Bumper -> Enable Slow Mode While Held
    m_driverController.rightBumper()
    .whileTrue(new InstantCommand(() -> m_robotDrive.setSlowMode(true)))
    .onFalse(new InstantCommand(() -> m_robotDrive.setSlowMode(false)));

    // Reef Scoring Alignments - d-pad aligns left, right, or center to reef sides using apriltag
    // D-pad Left -> Align to reef using pipeline 0 (LEFT alignment)
    m_driverController.povLeft().whileTrue(m_robotDrive.alignToReefCommand(Alignment.LEFT))
    .onFalse(new InstantCommand(() -> m_robotDrive.setSlowMode(false))); // ✅ Release slow mode

    // D-pad Right -> Align to reef using pipeline 1 (RIGHT alignment)
    m_driverController.povRight().whileTrue(m_robotDrive.alignToReefCommand(Alignment.RIGHT))
    .onFalse(new InstantCommand(() -> m_robotDrive.setSlowMode(false))); // ✅ Release slow mode

        // D-pad Right -> Align to reef using pipeline 1 (RIGHT alignment)
        m_driverController.povUp().whileTrue(m_robotDrive.alignToReefCommand(Alignment.CENTER))
        .onFalse(new InstantCommand(() -> m_robotDrive.setSlowMode(false))); // ✅ Release slow mode


/////////////////////////////////////////////////////////////////////////////////////////////////////////
    
// Operator Controller
    // Reef Scoring Alignments - d-pad aligns left, right, or center to reef sides using apriltag

    // D-pad Left -> Align to reef using pipeline 0 (LEFT alignment)
    m_operatorController.povLeft().whileTrue(m_robotDrive.alignToReefCommand(Alignment.LEFT))
    .onFalse(new InstantCommand(() -> m_robotDrive.setSlowMode(false))); // ✅ Release slow mode

    // D-pad Right -> Align to reef using pipeline 1 (RIGHT alignment)
    m_operatorController.povRight().whileTrue(m_robotDrive.alignToReefCommand(Alignment.RIGHT))
    .onFalse(new InstantCommand(() -> m_robotDrive.setSlowMode(false))); // ✅ Release slow mode



    // B Button -> Elevator/Arm to human player position
    m_operatorController.b().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kFeederStation));
    
    // START Button -> Elevator/Arm to level 1 position
    m_operatorController.start().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel1));
    
    // A Button -> Elevator/Arm to level 2 position
    m_operatorController.a().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel2));

    // X Button -> Elevator/Arm to level 3 position
    m_operatorController.x().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel3));
    
    // Operator option to run coral intake
    m_operatorController.rightBumper().whileTrue(m_coralSubSystem.runIntakeCommand());

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
