package frc.robot.subsystems;

import java.security.Key;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;

public class Limelight extends SubsystemBase{
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public Limelight(){
        SmartDashboard.putNumber("Distance", distanceFromTag());
        SmartDashboard.putNumber("Tx", getTx());
        SmartDashboard.putNumber("Ty", getTy());
        SmartDashboard.putNumber("Ta", getTa());
    }

    public static double getTx(){
        return table.getEntry("tx").getDouble(0);
    }

    public static double getTy(){
        return table.getEntry("ty").getDouble(0);
    }

    public static double getTa(){
        return table.getEntry("ta").getDouble(0);
    }

    public static boolean getTv(){
        return table.getEntry("tv").getDouble(0) > 0;
    }

    public static double distanceFromTag(){
        // Angle between floor and april tag in radians
        double angle = Math.toRadians(Vision.LLAngle + getTy());
        // Difference between the height of barge tag and limelight
        double heightDiff = Vision.bargeTagHeight - Vision.LLHeight;
        return heightDiff / Math.tan(angle);
    }
}
