// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

//import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.units.Units;  //added for fix
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
        public static class OperatorConstants {
          public static final int kDriverControllerPort = 0;
          public static final int kOperatorControllerPort = 1;
      
          public static final int kIntakePWM  = 0;
          public static final int kShooterPWM = 1;
          public static final int kPrimer1PWM = 2;
          public static final int kPrimer2PWM = 3;
          
          public static final double kPrimerSpeaker = 1;//0.80;  //was .50
          public static final double kPrimerAmp = 0.26;
          //0.99;  //0.26 IS THE BEST kPrimerAmp Value!!!
          public static final double kIntakeMotor = 1.0;
          public static final double kShooterMotor = 1.0;
        }
      

      public static final class ModuleConstants {
        //public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kWheelDiameterMeters = 0.1016;  //Value in Meters

       //public static final double kDriveMotorGearRatio = 1 / 5.8462; //Changed for mk4i L2 Ratio
        //public static final double kTurningMotorGearRatio = 1 / 18.0;  //Changed for mk4i L2 Ratio

        public static final double kDriveMotorGearRatio = 1 / 6.750;
        public static final double kTurningMotorGearRatio = 1 / 21.5285714;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = .15;  //Change to  1 for Experiment 2/15/24 12PM
    }

    public static final class DriveConstants {
        // Distance between right and left wheels
        //public static final double kTrackWidth = Units.inchesToMeters(21);
        public static final double kTrackWidth = 0.6096;  //Value in Meters  changed from original 0.5334
        //OUr Robots Track Width is 24" Exactly 0.6096

        // Distance between front and back wheels
        //public static final double kWheelBase = Units.inchesToMeters(25.5);
        public static final double kWheelBase = 0.6096;  //Value in Meters  changd from 0.6447
      //OUr Robots Track Wheelbase is 24" Exactly 0.6096
        
        //MUST MAP OUR ROBOT'S CAN IDS FOR MOTORS AND ENCODERS!!!


//MAKE the Translatoin 2d statements into Constants.
        public static final Translation2d kFrontLeftModulePosition = new Translation2d(kWheelBase / 2, kTrackWidth / 2);
        public static final Translation2d kFrontRightModulePosition = new Translation2d(kWheelBase / 2, -kTrackWidth / 2);
        public static final Translation2d kBackLeftModulePosition = new Translation2d(-kWheelBase / 2, kTrackWidth / 2);
        public static final Translation2d kBackRightModulePosition = new Translation2d(-kWheelBase / 2, -kTrackWidth / 2); 



                public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                        kFrontLeftModulePosition,
                        kFrontRightModulePosition,
                        kBackLeftModulePosition,
                        kBackRightModulePosition
                );

        // SAMPLE CODE BELOW
                // Creating my kinematics object using the module locations
                //use kDriveKinematics instead of m_kinematics
        // SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        //         m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
        // );


        //REPLACED BELOW CODE WITH ABOVE CODE TO CREATE MODLUE POSITION VARIABLES FOR kDriveKinematics
                // new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                // new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                // new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                // new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

                /*
                 new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

                 * 
                 */



        /*  ORIGINAL CAN MOTOR SETTINGS
        public static final int kFrontLeftDriveMotorPort = 8;
        public static final int kBackLeftDriveMotorPort = 2;
        public static final int kFrontRightDriveMotorPort = 6;
        public static final int kBackRightDriveMotorPort = 4;

        public static final int kFrontLeftTurningMotorPort = 7;
        public static final int kBackLeftTurningMotorPort = 1;
        public static final int kFrontRightTurningMotorPort = 5;
        public static final int kBackRightTurningMotorPort = 3;*/
        public static final int kFrontLeftDriveMotorPort = 2;
        public static final int kFrontRightDriveMotorPort = 4;
        public static final int kBackLeftDriveMotorPort = 6;
        public static final int kBackRightDriveMotorPort = 8;

        public static final int kFrontLeftTurningMotorPort = 3;
        public static final int kFrontRightTurningMotorPort = 5;
        public static final int kBackLeftTurningMotorPort = 7;
        public static final int kBackRightTurningMotorPort = 9;


        //public static final boolean kFrontLeftdriveMotorReversed = true;  //EXPERIMENTAL


        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;

        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;  //the right combination got us forward and backwas correct  but right pulling apart left driving was going into each other



//DriveEncoderReverse IS THE MOTOR REVERSE PARAMETER!!!!  //CHANGED TO TRUE TO MAKE ALL 4 SPIN FWD??
        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true; 
                                                                                //Changed to False for Turning Motors #3 Fix 2/12/24 Didn't Work
 
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = true;  
       // public static final boolean kBackRightDriveEncoderReversed = true;  //Changed to True for Drive Fix 2/8/24
        //public static final boolean kFrontRightDriveEncoderReversed = true;


        /*  ORIGINAL ENCODER CAN SETTINGS
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
        public static final int kBackRightDriveAbsoluteEncoderPort = 3;*/

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 10;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 11;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 12;
        public static final int kBackRightDriveAbsoluteEncoderPort = 13;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        //public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -0.254;
        //public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -1.252;
        //public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -1.816;
        //public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -4.811;
//RESET OFFSETS TO 0 TO TROUBLESHOOT DRIVE ISSUE 2/14/24
        // public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad =  0.113514578;  // -0.1165825;
        // public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 2.049398333;  //1.3100190;
        // public static final double kBackLeftDriveAbsoluteEncoderOffsetRad =   1.802427426;  //-2.0539984;
        // public static final double kBackRightDriveAbsoluteEncoderOffsetRad =  -3.132388769;  //-1.8131652;


        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0;//-2.041727349;// -0.1745329252;  //2.6355;//-5.271;//-1.570795;//.720808;//-0.27919244; //BR//0.03489854;//-0.2443539;//-1.81514089;// -4.79288784;  //-2.024581932;//-1.27;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad =  0;// -1.785555601; //0.907571211;  //-1.570795; //0;//0.8735483;//1.62315483;  //FR//1.570795; //-6.244460814;//-1.8675023;//-1.27;  //Changed to correct for being off by 90 degrees


        public static final double kPhysicalMaxSpeedMetersPerSecond = 7;  // Original Value 5, Less increases max speed, more reduces max speed.

        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;  //ADDED FOR 2nd Joystick Operator Control
        

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 2; //Changed from 4 to 2 to make joystick Z axis Rotation Axis
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.05;
        public static final double kRotationDeadband = 0.05;
    }
}
