package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
//import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ModuleConstants;

public final class Configs {
  public static final class MAXSwerveModule {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {

      // Use module constants to calculate conversion factors and feed forward gain.
      double drivingFactor = 
          ModuleConstants.kWheelDiameterMeters * Math.PI / ModuleConstants.kDrivingMotorReduction;
      double turningFactor = 2 * Math.PI;
      double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

      drivingConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
      drivingConfig
          .encoder
          .positionConversionFactor(drivingFactor) // meters
          .velocityConversionFactor(drivingFactor / 60.0); // meters per second
      drivingConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(0.04, 0, 0.003)
          .velocityFF(drivingVelocityFeedForward)
          .outputRange(-1, 1);

      turningConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
      turningConfig
          .absoluteEncoder
          // Invert the turning encoder, since the output shaft rotates in the opposite
          // direction of the steering motor in the MAXSwerve Module.
          .inverted(true)
          .positionConversionFactor(turningFactor) // radians
          .velocityConversionFactor(turningFactor / 60.0); // radians per second
      turningConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(1, 0, 0)
          .outputRange(-1, 1)
          // Enable PID wrap around for the turning motor. This will allow the PID
          // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
          // to 10 degrees will go through 0 rather than the other direction which is a
          // longer route.
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, turningFactor);
    }
  }

  public static final class CoralSubsystem {
    public static final SparkMaxConfig armConfig = new SparkMaxConfig();
    public static final SparkMaxConfig armFastConfig = new SparkMaxConfig();
    public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    public static final SparkFlexConfig intakeConfig = new SparkFlexConfig();
    public static final SparkMaxConfig frontIntakeConfig = new SparkMaxConfig();
    static {
      // Configure basic settings of the arm motor
      armConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
      armFastConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      armConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .p(0.35)
          .d(0.001)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(4000)
          .maxAcceleration(4000)
          .allowedClosedLoopError(0.250);

      armFastConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .p(0.35) // same P gain
          .d(0.001) // same D gain
          .outputRange(-1, 1)
          .maxMotion
          .maxVelocity(10000) // 🔥 FASTER for tossing
          .maxAcceleration(10000) // 🔥 FASTER for tossing
          .allowedClosedLoopError(0.250);

        

      // Configure basic settings of the elevator motor
      elevatorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(50).voltageCompensation(12);

      /*
       * Configure the reverse limit switch for the elevator. By enabling the limit switch, this
       * will prevent any actuation of the elevator in the reverse direction if the limit switch is
       * pressed.
       */
      // elevatorConfig
      //     .limitSwitch
      //     .reverseLimitSwitchEnabled(true)
      //     .reverseLimitSwitchType(Type.kNormallyOpen);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      elevatorConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .p(0.2)
          .d(0.0015)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(58000)
          .maxAcceleration(60000)
          .allowedClosedLoopError(0.5);

            /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      elevatorConfig
          .softLimit
          .reverseSoftLimitEnabled(true)
          .reverseSoftLimit(1)
          .forwardSoftLimitEnabled(true)
          .forwardSoftLimit(148); // set limit slightly below hard limit of 150 to ensure safe stop



      // Configure basic settings of the intake motor
      intakeConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
      frontIntakeConfig.inverted(true).idleMode(IdleMode.kCoast).smartCurrentLimit(40);
    }
  }

  public static final class AlgaeSubsystem {
    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();
    static {
      // Configure basic settings of the intake motor
      intakeConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    }
  }

  public static final class ClimberSubsystem {
    public static final SparkMaxConfig climbArmConfig = new SparkMaxConfig();
    public static final SparkMaxConfig climbConfig = new SparkMaxConfig();

    static {
      // Configure basic settings of the arm motor
      climbArmConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      climbArmConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control
          .p(0.1)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(12000)
          .maxAcceleration(12000)
          .allowedClosedLoopError(0.15);
    }
}
}
