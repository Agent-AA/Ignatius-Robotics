// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

//   REPLACED BY ALTERNATE IMPORTS (SEE BELOW)
//import edu.wpi.first.wpilibj.controller.PIDController;
//import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
//import edu.wpi.first.wpilibj.geometry.Pose2d;
//import edu.wpi.first.wpilibj.geometry.Rotation2d;
//import edu.wpi.first.wpilibj.geometry.Translation2d;
//import edu.wpi.first.wpilibj.trajectory.Trajectory;
//import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
//import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ShootingIntakeCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;  //FOR FLIGHT STICK
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.ShootingSubsystem;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final ShootingSubsystem shootingSubsystem = new ShootingSubsystem();
    public final static ColorSensorSubsystem colorSensorSubsystem = new ColorSensorSubsystem();

  //SWITCHING TO FLIGHTSTICK  CONTROLLER  
//     private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort); //0
//     private final Joystick operatorJoytick = new Joystick(OIConstants.kOperatorControllerPort); //0

    //private final CommandJoystick commanddriverJoytick = new CommandJoystick(OIConstants.kDriverControllerPort);
//SWITCHING TO XBOX CONTROLLER
    private final XboxController xboxDriver = new XboxController(OIConstants.kDriverControllerPort);
    private final XboxController xboxController = new XboxController(Constants.OperatorConstants.kOperatorControllerPort); //1
    
    //private final XboxController operatorController = new XboxController(Constants.OperatorConstants.kOperatorControllerPort); //1

    //private final ShootingSubsystem m_shootingSubsystem = ShootingSubsystem.getInstance();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    //HAD TO INVERT X AND Z AXIS TO GET TO DRIVE ROBOT
    swerveSubsystem.setDefaultCommand(
        
                new SwerveJoystickCmd(
                swerveSubsystem,
                shootingSubsystem,
    //SWITCHING TO FLIGHTSTICK  CONTROLLER            
                // () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis), //1
                // () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),  //0  //Placed a negative in front of X axis to fix issue with going in opposite right / left direction
                // () -> -operatorJoytick.getRawAxis(OIConstants.kDriverRotAxis), //3  CHANGE to 2nd JOYSTICK
                // () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));
//SWITCHING TO XBOX CONTROLLER
                () -> -xboxDriver.getRawAxis(OIConstants.kDriverYAxis), //1
                () -> -xboxDriver.getRawAxis(OIConstants.kDriverXAxis),  //0  //Placed a negative in front of X axis to fix issue with going in opposite right / left direction
                () -> -xboxDriver.getRawAxis(4), //4  CHANGE to 2nd JOYSTICK of Xbox Controller
                () -> !xboxDriver.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
                () -> xboxDriver.getRawButton(2),
                () -> xboxDriver.getRawButton(3),
                () -> xboxDriver.getRawButton(4),  //ADDED FOR A BUTTON TO TURN 90 Degrees CW
//ADDED 2ND XBOX CONTROLLER FOR OPERATOR CONTROLLER TO CONTROL INTAKE AND SHOOTER 

                () -> xboxDriver.getRawButton(9),     // Added for Drive Turbo
                
                
                () -> xboxController.getRawButton(4),  //XboxController #2 "Y" Button (#4) for SHOOTING AT SPEAKER
                () -> xboxController.getRawButton(1),  //XboxController #2 "A" Button (#1) for SHOOTING AT AMP
                () -> xboxController.getRawButton(5),
                () -> xboxController.getRawButton(6)
                ));  

               

                

        configureButtonBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */


  private void configureButtonBindings() {
    //NEED ONE OF THE NEXT TWO LINES TO WORK
    //This is supposed to set the 2nd joystick button on the main joystick
    //new JoystickButton(driverJoytick, 2).whenPressed(() -> swerveSubsystem.zeroHeading()); //ORIGINAL
    //new JoystickButton(driverJoytick, 2).whileTrue(getAutonomousCommand())(() -> swerveSubsystem.zeroHeading());

        // Trigger leftBumper = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value)
        // .onTrue(m_shootingSubsystem.togglePriming(1));

        // Trigger rightBumper = new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value)
        // .onTrue(m_shootingSubsystem.togglePriming(0));

        // Trigger aButton = new JoystickButton(operatorController, XboxController.Button.kA.value)
        // .onTrue(m_shootingSubsystem.shoot())
        // .onFalse(m_shootingSubsystem.unShoot());
}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

        // An example command will be run in autonomous
        return(null); //MUST REMOVE THIS LINE ONCE AUTONOMOUS IS IN
  
    // BEN'S CODE:   
    //return Commands.Run(() -> swerveSubsystem.setModuleStates({0.5,0.5,0.5,0.5,0.5}))
    
    //ADD BELOW CODE FOR AUTONOMOUS BEGIN HERE  
 /* 
    
    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DriveConstants.kDriveKinematics);

        //TrajectoryConfig trajectoryConfig = new TrajectoryConfig(1.25, 3);

        // 2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(1, 0),
                        new Translation2d(1, -1)),
                new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
                trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
// NEED TO FIX
        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));

                 
       //ADD ABOVE CODE FOR AUTONOMOUS END HERE  
*/                  
    }
  
}
