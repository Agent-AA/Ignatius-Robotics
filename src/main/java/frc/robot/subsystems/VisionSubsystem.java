package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase {  
    
    NetworkTable m_limelightTable;
    NetworkTableEntry tv, tx, ty, ta;
    public static double nt_xOffset, nt_yOffset, nt_area;
    public static boolean nt_visibility;
    
    public VisionSubsystem() {
        NetworkTable m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableInstance.getDefault().startServer();
        NetworkTableInstance.getDefault().setServerTeam(5409);   
        
        tv = m_limelightTable.getEntry("tv");
        tx = m_limelightTable.getEntry("tx");
        ty = m_limelightTable.getEntry("ty");
        ta = m_limelightTable.getEntry("ta");
    }

    @Override
    public void periodic() {
        updateAprilPose();
    }     
    public void updateAprilPose() {
        // Read values
        nt_visibility = tv.getBoolean(false);
        nt_xOffset = tx.getDouble(0.0);
        nt_yOffset = ty.getDouble(0.0);
        nt_area = ta.getDouble(0.0);
        
        // Post to smart dashboard
        SmartDashboard.putBoolean("LimelightTargeting?", nt_visibility);
        SmartDashboard.putNumber("LimelightX", nt_xOffset);
        SmartDashboard.putNumber("LimelightY", nt_yOffset);
        SmartDashboard.putNumber("LimelightArea", nt_area);
    }

}
