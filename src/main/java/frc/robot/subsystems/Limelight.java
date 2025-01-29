package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;

public class Limelight extends SubsystemBase{
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public Limelight(){
        
    }

    public static double getTx(){
        return table.getEntry("tx").getDouble(0);
    }

    public static double getTy(){
        return table.getEntry("ty").getDouble(0);
    }

    public static boolean getTv(){
        return table.getEntry("tv").getDouble(0) > 0;
    }

    public static double distanceFromTag(){
        // Angle between floor and april tag in radians
        double angle = (Vision.LLAngle + getTy()) * (Math.PI / 180);
        // Difference between the height of barge tag and limelight
        double heightDiff = Vision.bargeTagHeight - Vision.LLHeight;
        return heightDiff / Math.tan(angle);
    }
}
