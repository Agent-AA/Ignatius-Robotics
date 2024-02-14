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
    //private final ADIS16448_IMU ADISgyro = new ADIS16448_IMU();
    //private final double moduleDistance = Math.sqrt(Math.pow(DriveConstants.kWheelBase / 2, 2) + Math.pow(DriveConstants.kTrackWidth / 2, 2));
    //private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
        // DriveConstants.kDriveKinematics, 
        // getRotation2d(), 
        // new SwerveModulePosition[] {
        //     new SwerveModulePosition(moduleDistance, frontLeft.getState().angle),
        //     new SwerveModulePosition(moduleDistance, frontRight.getState().angle),
        //     new SwerveModulePosition(moduleDistance, backLeft.getState().angle),
        //     new SwerveModulePosition(moduleDistance, backRight.getState().angle)
        // });
Pigeon2 P2gyro = new Pigeon2(20);  //Using CANID #20
// public void zeroHeading() {
//   p2gyro.reset();
// }

// public double getHeading() {
//   return Math.IEEEremainder(p2gyro.getAngle(), 360);
// }
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

/*
    //COMMENTED OUT ODOMETER CODE - NEED TO FIGURE OUT
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }
///NEED TO RESOLVE ERROR:  ﻿4:47:45.252 PM
// Error at frc.robot.subsystems.SwerveSubsystem.resetOdometry(SwerveSubsystem.java:105): Unhandled exception: java.lang.IllegalArgumentException: Number of modules is not consistent with number of wheel locations provided in constructor ERROR  1  Unhandled exception: java.lang.IllegalArgumentException: Number of modules is not consistent with number of wheel locations provided in constructor  frc.robot.subsystems.SwerveSubsystem.resetOdometry(SwerveSubsystem.java:105) 

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), new SwerveModulePosition[0], pose);
    }

    */
    @Override
    public void periodic() {
        // odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
        
         //SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

         SmartDashboard.putNumber("Robot Heading", Math.round(getHeading()));

        
         
    }



    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
        
    }
 //Sets each of the SwerveModules (FL,FR,BL,BR) to appropriate SwerveModule State
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);   
        //Above changed from:  SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
         //frontLeft.setDesiredState(desiredStates[0]);  //not sure why there's a 2nd line of this same code?

        //USE THIS CODE TO MAYBE MANUALLY SET SPEED AND TURN TO EACH SWERVE DRIVE?

        // Example chassis speeds: 1 meter per second forward, 3 meters
// per second to the left, and rotation at 1.5 radians per second
// counterclockwise.

    // }
    //  public void setModuleStates2(SwerveModuleState[] desiredStates) {

    //         ChassisSpeeds speeds = new ChassisSpeeds(1.0, 3.0, 1.5);

    //         // Convert to module states
    //         SwerveModuleState[] moduleStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    //         // Front left module state
    //         SwerveModuleState swervemodulefrontLeft2 = moduleStates[0];
    //         // Front right module state
    //         SwerveModuleState swervemodulefrontRight2 = moduleStates[1];
    //         // Back left module state
    //         SwerveModuleState swervemodulebackLeft2 = moduleStates[2];
    //         // Back right module state
    //         SwerveModuleState swervemodulebackRight2 = moduleStates[3];

    //    // swervemodulefrontLeft2.angle;
    }
    
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
    }
}