package frc.robot.subsystems;

//import com.kauailabs.navx.frc.AHRS;  Using Analog Devices ADIS16448 Gyro instead (see below)
import edu.wpi.first.wpilibj.ADIS16448_IMU;

import edu.wpi.first.wpilibj.SPI;
/*   REPLACED BY ALTERNATE IMPORTS (SEE BELOW)
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
*/

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.WheelPositions;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule( 
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
           // DriveConstants.kFrontLeftdriveMotorReversed,   //EXPERIMENTAL
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

   // private final AHRS gyro = new AHRS(SPI.Port.kMXP);  //REPLACED WITH ADIS16448_IMU see below
    private final ADIS16448_IMU gyro = new ADIS16448_IMU();


    // Odometry class for tracking robot pose
    SwerveDriveOdometry odometer =
        new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(gyro.getAngle()), // Convert gyro degrees to Rotation2d
            new SwerveModulePosition[] { // Getting distance & rotation2d from each swervemodule
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            }); // May need to add a Pose2d for our starting pose
    // Odometry class for tracking robot pose
    SwerveDriveOdometry odometer =
        new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(gyro.getAngle()), // Convert gyro degrees to Rotation2d
            new SwerveModulePosition[] { // Getting distance & rotation2d from each swervemodule
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            }); // May need to add a Pose2d for our starting pose

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        P2gyro.reset();
    }

    public double getHeading() {
        return -Math.IEEEremainder(P2gyro.getAngle(), 360);  //Placed Negative since Pigeon2 is CCW+ and WPLIB assumes Gyros are CW+
        //FROM CHIEF DELPHI POST  "At headings of 0 and 180 degrees, everything functioned as expected (i.e. up on the joystick was field-forward, left on joystick was field-left, etc.) but at 90 and 270 degree headings things seemed to get inverted (joystick-left was field-right, joystick-up was field-down)."
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    // Fixed odometer issue so that I can use odometry with vision
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }
///NEED TO RESOLVE ERROR:  ﻿4:47:45.252 PM
// Error at frc.robot.subsystems.SwerveSubsystem.resetOdometry(SwerveSubsystem.java:105): Unhandled exception: java.lang.IllegalArgumentException: Number of modules is not consistent with number of wheel locations provided in constructor ERROR  1  Unhandled exception: java.lang.IllegalArgumentException: Number of modules is not consistent with number of wheel locations provided in constructor  frc.robot.subsystems.SwerveSubsystem.resetOdometry(SwerveSubsystem.java:105) 
*/
    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(
            Rotation2d.fromDegrees(gyro.getAngle()), // Convert gyro degrees to Rotation2d
            new SwerveModulePosition[] {
              frontLeft.getPosition(),
              frontRight.getPosition(),
              backLeft.getPosition(),
              backRight.getPosition()
            },
            pose);
        odometer.resetPosition(
            Rotation2d.fromDegrees(gyro.getAngle()), // Convert gyro degrees to Rotation2d
            new SwerveModulePosition[] {
              frontLeft.getPosition(),
              frontRight.getPosition(),
              backLeft.getPosition(),
              backRight.getPosition()
            },
            pose);
    }

    
    @Override
    public void periodic() {
        // Update the odometry
        odometer.update(
            Rotation2d.fromDegrees(gyro.getAngle()),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            });
        // Update the odometry
        odometer.update(
            Rotation2d.fromDegrees(gyro.getAngle()),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            });
        SmartDashboard.putNumber("Robot Heading", getHeading());
        //SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
 
  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);   //MUST FIX!
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond); // Fixed
        // SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);   //MUST FIX!
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond); // Fixed
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
    
    // Custom drive function for lining up with limelight and for troubleshooting
    // does what SwerveJoystickCmd does, but takes in a speed and not a speed supplier.
    // Custom drive function for lining up with limelight and for troubleshooting
    // does what SwerveJoystickCmd does, but takes in a speed and not a speed supplier.
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        var swerveModuleStates =
            DriveConstants.kDriveKinematics.toSwerveModuleStates(
                ChassisSpeeds.discretize(
                    fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, this.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                0.02)); // 0.02 = once per scheduler call
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }
}