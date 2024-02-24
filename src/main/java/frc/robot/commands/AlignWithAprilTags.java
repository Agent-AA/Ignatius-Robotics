package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem; 
import frc.robot.subsystems.VisionSubsystem; 
import edu.wpi.first.wpilibj2.command.Command;

public class AlignWithAprilTags extends Command {
    private final SwerveSubsystem m_swerveSubsystem;
    private final VisionSubsystem m_visionSubsystem;

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
    
    }

    @Override
    public void execute() {

    }
    
    
}
