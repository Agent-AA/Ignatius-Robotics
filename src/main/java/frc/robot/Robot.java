// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.revrobotics.CANSparkLowLevel;
/*BEGIN YAGSL ENCODER FEEDBACK CODE */
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import edu.wpi.first.math.util.Units;
import java.lang.Math;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.cameraserver.CameraServer;



/*END YAGSL ENCODER FEEDBACK CODE */  //FOUND AT FUNDAMETALS--> SWERVE MODULES
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;






//BEGIN CODE TO OUTPUT DRIVE AND TURNING MOTOR ENCODER VALUES

// //These are created to display realtime feedback of Drive and Turn Encoders
// CANSparkMax FLdriveMotor = new CANSparkMax(2,CANSparkLowLevel.MotorType.kBrushless); 
// CANSparkMax FLturningMotor = new CANSparkMax(3,CANSparkLowLevel.MotorType.kBrushless);
// CANSparkMax FRdriveMotor = new CANSparkMax(4,CANSparkLowLevel.MotorType.kBrushless);
// CANSparkMax FRturningMotor = new CANSparkMax(5,CANSparkLowLevel.MotorType.kBrushless);
// CANSparkMax BLdriveMotor = new CANSparkMax(6,CANSparkLowLevel.MotorType.kBrushless);
// CANSparkMax BLturningMotor = new CANSparkMax(7,CANSparkLowLevel.MotorType.kBrushless);
// CANSparkMax BRdriveMotor = new CANSparkMax(8,CANSparkLowLevel.MotorType.kBrushless);
// CANSparkMax BRturningMotor = new CANSparkMax(9,CANSparkLowLevel.MotorType.kBrushless);

// RelativeEncoder FLdriveEncoder = FLdriveMotor.getEncoder();
// RelativeEncoder FLturningEncoder = FLturningMotor.getEncoder();
// RelativeEncoder FRdriveEncoder = FRdriveMotor.getEncoder();
// RelativeEncoder FRturningEncoder = FRturningMotor.getEncoder();
// RelativeEncoder BLdriveEncoder = BLdriveMotor.getEncoder();
// RelativeEncoder BLturningEncoder = BLturningMotor.getEncoder();
// RelativeEncoder BRdriveEncoder = BRdriveMotor.getEncoder();
// RelativeEncoder BRturningEncoder = BRturningMotor.getEncoder();
/*BEGIN YAGSL ENCODER FEEDBACK CODE */

    private static Robot   instance;
  private CANcoder    cancoderFL;
  private CANcoder    cancoderFR;
  private CANcoder    cancoderBL;
  private CANcoder    cancoderBR;
  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

/*END YAGSL ENCODER FEEDBACK CODE */

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    CameraServer.startAutomaticCapture();


    /*BEGIN YAGSL ENCODER FEEDBACK CODE */
    cancoderFL = new CANcoder(/* Change this to the CAN ID of the CANcoder */ 10);
    cancoderFR = new CANcoder(/* Change this to the CAN ID of the CANcoder */ 11);
    cancoderBL = new CANcoder(/* Change this to the CAN ID of the CANcoder */ 12);
    cancoderBR = new CANcoder(/* Change this to the CAN ID of the CANcoder */ 13);
   CANcoderConfigurator cfgFL = cancoderFL.getConfigurator();
   cfgFL.apply(new CANcoderConfiguration());
   MagnetSensorConfigs  magnetSensorConfigurationFL = new MagnetSensorConfigs();
   cfgFL.refresh(magnetSensorConfigurationFL);
   cfgFL.apply(magnetSensorConfigurationFL
                  .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                  .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

   CANcoderConfigurator cfgFR = cancoderFR.getConfigurator();
   cfgFR.apply(new CANcoderConfiguration());
   MagnetSensorConfigs  magnetSensorConfigurationFR = new MagnetSensorConfigs();
   cfgFR.refresh(magnetSensorConfigurationFR);
   cfgFR.apply(magnetSensorConfigurationFR
                  .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                  .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

   CANcoderConfigurator cfgBL = cancoderFL.getConfigurator();
   cfgBL.apply(new CANcoderConfiguration());
   MagnetSensorConfigs  magnetSensorConfigurationBL = new MagnetSensorConfigs();
   cfgBL.refresh(magnetSensorConfigurationBL);
   cfgBL.apply(magnetSensorConfigurationBL
                  .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                  .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

   CANcoderConfigurator cfgBR = cancoderBR.getConfigurator();
  cfgBR.apply(new CANcoderConfiguration());
   MagnetSensorConfigs  magnetSensorConfigurationBR = new MagnetSensorConfigs();
   cfgBR.refresh(magnetSensorConfigurationBR);
   cfgBR.apply(magnetSensorConfigurationBR
                  .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                  .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));
                  
  /*END YAGSL ENCODER FEEDBACK CODE */
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    RobotContainer.colorSensorSubsystem.displayColors();


/*BEGIN Drive and Turning Motor Encoder FEEDBACK CODE */
// FLdriveMotor.setInverted(Constants.DriveConstants.kFrontLeftDriveEncoderReversed);
// FLturningEncoder.setInverted(Constants.DriveConstants.kFrontLeftTurningEncoderReversed);
// FRdriveMotor.setInverted(Constants.DriveConstants.kFrontRightDriveEncoderReversed);
// FRturningEncoder.setInverted(Constants.DriveConstants.kFrontRightTurningEncoderReversed);
// BLdriveMotor.setInverted(Constants.DriveConstants.kBackLeftDriveEncoderReversed);
// BLturningEncoder.setInverted(Constants.DriveConstants.kBackLeftDriveEncoderReversed);
// BRdriveMotor.setInverted(Constants.DriveConstants.kBackRightDriveEncoderReversed);
// BRturningEncoder.setInverted(Constants.DriveConstants.kBackRightDriveEncoderReversed);

// FLdriveEncoder=FLdriveMotor.getEncoder();
// FLturningEncoder=FLturningMotor.getEncoder();
// FRdriveEncoder=FRdriveMotor.getEncoder();
// FRturningEncoder=FRturningMotor.getEncoder();
// BLdriveEncoder=BLdriveMotor.getEncoder();
// BLturningEncoder=BLturningMotor.getEncoder();
// BRdriveEncoder=BRdriveMotor.getEncoder();
// BRturningEncoder=BRturningMotor.getEncoder();

// FLdriveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
// FLdriveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
// FLdriveEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
// FLdriveEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
// FRdriveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
// FRdriveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
// FRdriveEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
// FRdriveEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
// BLdriveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
// BLdriveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
// BLdriveEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
// BLdriveEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
// BRdriveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
// BRdriveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
// BRdriveEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
// BRdriveEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
/*END Drive and Turning Motor Encoder FEEDBACK CODE */

/*BEGIN YAGSL ENCODER FEEDBACK CODE */
//Absolute Angle
    StatusSignal<Double> angleFL = cancoderFL.getAbsolutePosition().waitForUpdate(0.05);
    StatusSignal<Double> angleFR = cancoderFR.getAbsolutePosition().waitForUpdate(0.05);
    StatusSignal<Double> angleBL = cancoderBL.getAbsolutePosition().waitForUpdate(0.05);
    StatusSignal<Double> angleBR = cancoderBR.getAbsolutePosition().waitForUpdate(0.05);
//Position Angle 
//SWITCH TO POSTITION SINCE BOOT
    // StatusSignal<Double> PangleFL = cancoderFL.getPosition().waitForUpdate(0.05);
    // StatusSignal<Double> PangleFR = cancoderFR.getPosition().waitForUpdate(0.05);
    // StatusSignal<Double> PangleBL = cancoderBL.getPosition().waitForUpdate(0.05);
    // StatusSignal<Double> PangleBR = cancoderBR.getPosition().waitForUpdate(0.05);
        StatusSignal<Double> PangleFL = cancoderFL.getPositionSinceBoot().waitForUpdate(0.05);
    StatusSignal<Double> PangleFR = cancoderFR.getPositionSinceBoot().waitForUpdate(0.05);
    StatusSignal<Double> PangleBL = cancoderBL.getPositionSinceBoot().waitForUpdate(0.05);
    StatusSignal<Double> PangleBR = cancoderBR.getPositionSinceBoot().waitForUpdate(0.05);


    //StatusSignal<Double> angleXX = cancoderBR.
  System.out.println("AbsolutePosition----------------------------");
    System.out.println("angleFL Absolute Encoder Angle getAbsolutePosition(degrees): " + Math.round(Units.rotationsToDegrees(angleFL.getValue())));//+  "      (radians): "+ Units.rotationsToRadians(angleFL.getValue()));
    System.out.println("angleFR Absolute Encoder Angle getAbsolutePosition(degrees): " + Math.round(Units.rotationsToDegrees(angleFR.getValue())));// + "      (radians): "+ Units.rotationsToRadians(angleFR.getValue()));
    System.out.println("angleBL Absolute Encoder Angle getAbsolutePosition(degrees): " + Math.round(Units.rotationsToDegrees(angleBL.getValue())));// + "      (radians): "+ Units.rotationsToRadians(angleBL.getValue()));
    System.out.println("angleBR Absolute Encoder Angle getAbsolutePosition(degrees): " + Math.round(Units.rotationsToDegrees(angleBR.getValue())));// + "      (radians): "+ Units.rotationsToRadians(angleBR.getValue()));

  System.out.println("---------------------Position SINCE BOOT!---------------------------------------------------------------");
    System.out.println("angleFL Absolute Encoder Angle Position(degrees): " + Math.round(Units.rotationsToDegrees(PangleFL.getValue())));
    System.out.println("angleFR Absolute Encoder Angle Position(degrees): " + Math.round(Units.rotationsToDegrees(PangleFR.getValue())));
    System.out.println("angleBL Absolute Encoder Angle Position(degrees): " + Math.round(Units.rotationsToDegrees(PangleBL.getValue())));
    System.out.println("angleBR Absolute Encoder Angle Position(degrees): " + Math.round(Units.rotationsToDegrees(PangleBR.getValue())));

    //System.out.println("Pigeon 2 Gyro Reading:  " + Math.round(SwerveSubsystem.getHeading()));

   // System.out.println("FRONT LEFT DRIVE MOTOR With CANID[" + FLdriveMotor.getDeviceId() + "]:  getEncoder value:  " + FLdriveMotor.getEncoder()  + ":  getEncoder.getPosition value:  " + FLdriveMotor.getEncoder().getPosition() + ":  getEncoder.getInverted value:  " + FLdriveMotor.getEncoder().getInverted() );
  // System.out.println("RADIANS----------------------------");
  //   System.out.println("angleFL Absolute Encoder Angle (radians): " + Units.rotationsToRadians(angleFL.getValue()));
  //   System.out.println("angleFR Absolute Encoder Angle (radians): " + Units.rotationsToRadians(angleFR.getValue()));
  //   System.out.println("angleBL Absolute Encoder Angle (radians): " + Units.rotationsToRadians(angleBL.getValue()));
  //   System.out.println("angleBR Absolute Encoder Angle (radians): " + Units.rotationsToRadians(angleBR.getValue()));
/*END YAGSL ENCODER FEEDBACK CODE */

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
