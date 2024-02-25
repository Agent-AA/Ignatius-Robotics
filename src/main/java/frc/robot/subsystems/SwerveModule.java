package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;   // Used com.revrobotics.CANSparkLowLevel.MotorType; instead
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.geometry.Rotation2d;
//import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;



public class SwerveModule {
    //private CANSparkMax m_SparkMaxMotor = new CANSparkMax(10, MotorType.kBrushless);
    //private final CANSparkMax m_CanSparkMax;
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    //private final CANcoder driveEncoder;   //replaced REV's CANencoder / RelativeEncoder with CTRE's CANcoder 
    //private final CANcoder turningEncoder; //replaced REV's CANencoder / RelativeEncoder with CTRE's CANcoder 

    private final RelativeEncoder driveEncoder;   //replaced REV's CANencoder with RevRobotics RelativeEncoder 
    private final RelativeEncoder turningEncoder; //replaced REV's CANencoder  with RevRobotics RelativeEncoder  

    private final PIDController turningPidController;

    //private final AnalogInput absoluteEncoder;  //AnalogInput Encoder Replaced by CANencoder absoluteEncoder  
    private final CANcoder absoluteEncoder;
   
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;
//IMPLEMENTION SWERVEMODULE 12:41
    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        //absoluteEncoder = new AnalogInput(absoluteEncoderId); //replaced by CTRE CANcoder
        absoluteEncoder = new CANcoder(absoluteEncoderId);
       

        //m_CanSparkMax = new CANSparkMax(10,MotorType.kBrushless);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        

        
        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

      

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

  // Added getPosition function to enable odometry with vision
    /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        // WILL NEED TWEAKING: I attempted to replicate what getDistance() does on an AbsoluteEncoder
        // for our RelativeEncoders
        driveEncoder.getPosition() * ModuleConstants.kDriveEncoderRot2Meter,
        new Rotation2d(turningEncoder.getPosition() * ModuleConstants.kTurningEncoderRot2Rad));
        // NB: the rotation2d is not accurate, as I don't know the diameter of the turning encoder
  }    
    
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

   /*  Replaced public double getAbsoluteEncoderRad() function with getAbsoluteEncoderPosition function AND resetEncoders with per the following:
   https://www.chiefdelphi.com/t/how-to-get-radians-voltage-from-ctre-cancoder/450759/3   due to using CANcoder vs. Analog encoder
   
   {

    public double getAbsoluteEncoderRad() {
        //double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        double angle = absoluteEncoder.getBus() / RobotController.getVoltage5V();

        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    }
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

   */

    public double getAbsoluteEncoderPosition() {
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderPosition());
    }    
    
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        //SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());  //Modified for CANcoder
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }


}
  