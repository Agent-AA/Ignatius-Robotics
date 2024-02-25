package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem; 
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Commands;

// Trajectory imports
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand; // To move robot along trajectory
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

// Note: this method of subclassing Command is relatively verbose
// Ideally I would have created a static command factory
// But making the command this way is easier to understand
public class AlignWithAprilTags extends Command {
    private final SwerveSubsystem m_swerveSubsystem;
    private final VisionSubsystem m_visionSubsystem;

    private double offsetX = 0;
    private double offsetY = 0;
    private double offsetA = 0;


    /**
     * Command constructor
     */
    public AlignWithAprilTags(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem) {
        m_swerveSubsystem = swerveSubsystem;
        m_visionSubsystem = visionSubsystem;
        // add Command requirements
        addRequirements(m_swerveSubsystem, m_visionSubsystem);
    }

    @Override
    public void initialize() {
        // Create config for tragectory
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared
        );

        // Where the robot should end up, based on target AprilTag
        Pose2d targetPose = new Pose2d(
            // target x
            // target y
            // target rotation
        ); 

        // Create path for robot to follow, starting from its current position
        // and ending with the AprilTag in a specific spot
        Trajectory visionTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, m_swerveSubsystem.getRotation2d()),
            null, // no interior waypoints for now
            targetPose,
            config);
    
        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);        

        // Create command that follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            visionTrajectory,
            m_swerveSubsystem::getPose, // Functional interface to feed supplier Pose2d
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_swerveSubsystem::setModuleStates, // The raw output module states from the position controllers
            m_swerveSubsystem
        );

        // Reset odometry to the initial pose of the trajectory, run path following
        // command, then stop at the end.
        Commands.sequence(
            new InstantCommand(() -> m_swerveSubsystem.resetOdometry(visionTrajectory.getInitialPose())),
            swerveControllerCommand,
            new InstantCommand(() -> m_swerveSubsystem.drive(0, 0, 0, false)));
    }

    @Override
    public boolean isFinished() {return true;} // Finishes command as soon as initialize() is over
}
