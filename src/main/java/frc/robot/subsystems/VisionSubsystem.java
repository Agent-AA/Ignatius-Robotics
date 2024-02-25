package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase {  
    
    public static double x, y, area;
    public static boolean v;
    
    public VisionSubsystem() {}

    @Override
    public void periodic() {
        NetworkTable m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tv = m_limelightTable.getEntry("tv");
        NetworkTableEntry tx = m_limelightTable.getEntry("tx");
        NetworkTableEntry ty = m_limelightTable.getEntry("ty");
        NetworkTableEntry ta = m_limelightTable.getEntry("ta");
        
        // Read values
        v = tv.getBoolean(false);
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
        
        // Post to smart dashboard
        SmartDashboard.putBoolean("LimelightTargeting?", v);
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
    }     
}
