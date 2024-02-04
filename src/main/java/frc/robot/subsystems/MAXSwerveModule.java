package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

// Constructor for creating modules that represent each wheel
public class MAXSwerveModule {
    // Variables for channels of the wheel. These are used in the constructor of this class.
    private final CANSparkMax m_drivingSparkMax;
    private final CANSparkMax m_turningSparkMax;
  
    private final RelativeEncoder m_drivingEncoder;
    private final AbsoluteEncoder m_turningEncoder;
  
    private final SparkPIDController m_drivePIDController;
    private final SparkPIDController m_turningPIDController; 
    
    /* Old code from WPI example swerve implementation
    private final PIDController m_drivePIDController =
        new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);
  
    // Using a TrapezoidProfile PIDController to allow for smooth turning
    private final ProfiledPIDController m_turningPIDController =
        new ProfiledPIDController(
            ModuleConstants.kPModuleTurningController,
            0,
            0,
            new TrapezoidProfile.Constraints(
                ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
                ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));
    */            
  
    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorChannel The channel of the drive motor.
     * @param turningMotorChannel The channel of the turning motor.
     * @param driveEncoderChannels The channels of the drive encoder.
     * @param turningEncoderChannels The channels of the turning encoder.
     * @param driveEncoderReversed Whether the drive encoder is reversed.
     * @param turningEncoderReversed Whether the turning encoder is reversed.
     */
    public MAXSwerveModule(int driveCANId, int turnCANId) {
      m_drivingSparkMax = new CANSparkMax(driveCANId, MotorType.kBrushless);
      m_turningSparkMax = new CANSparkMax(turnCANId, MotorType.kBrushless); //idk what the motor type is

      m_drivingSparkMax.restoreFactoryDefaults();
      m_turningSparkMax.restoreFactoryDefaults();    
    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
      m_drivingEncoder = m_drivingSparkMax.getEncoder();
      m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
      m_drivingPIDController = m_drivingSparkMax.getPIDController();
      m_turningPIDController = m_turningSparkMax.getPIDController();
      m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
      m_turningPIDController.setFeedbackDevice(m_turningEncoder);

      // Apply position and velocity conversion factors for the driving encoder. The
      // native units for position and velocity are rotations and RPM, respectively,
      // but we want meters and meters per second to use with WPILib's swerve APIs.
      m_drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
      m_drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

      // Apply position and velocity conversion factors for the turning encoder. We
      // want these in radians and radians per second to use with WPILib's swerve
      // APIs.
      m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
      m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

      // Invert the turning encoder, since the output shaft rotates in the opposite direction of
      // the steering motor in the MAXSwerve Module.
      m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

      // Enable PID wrap around for the turning motor. This will allow the PID
      // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
      // to 10 degrees will go through 0 rather than the other direction which is a
      // longer route.
      m_turningPIDController.setPositionPIDWrappingEnabled(true);
      m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
      m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    }
  
    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition()));
    }
  
    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
      return new SwerveModulePosition(
          m_drivingEncoder.getPosition(),
            new Rotation2d(m_turningEncoder.getPosition()));
    }
  
    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
      var encoderRotation = new Rotation2d(m_turningEncoder.getDistance());
  
      // Optimize the reference state to avoid spinning further than 90 degrees
      SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);
  
      // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
      // direction of travel that can occur when modules change directions. This results in smoother
      // driving.
      state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();
  
      // Calculate the drive output from the drive PID controller.
      final double driveOutput =
          m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);
  
      // Calculate the turning motor output from the turning PID controller.
      final double turnOutput =
          m_turningPIDController.calculate(m_turningEncoder.getDistance(), state.angle.getRadians());
  
      // Calculate the turning motor output from the turning PID controller.
      m_driveMotor.set(driveOutput);
      m_turningMotor.set(turnOutput);
    }
  
    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
      m_driveEncoder.reset();
      m_turningEncoder.reset();
    }
}
