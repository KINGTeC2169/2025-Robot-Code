package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;
import frc.robot.util.LimelightHelpers;

public class Limelight extends SubsystemBase{
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public Limelight(){
        SmartDashboard.putNumber("LL Tx", getTx());
        SmartDashboard.putNumber("LL Ty", getTy());
        SmartDashboard.putNumber("LL Ta", getTa());
        SmartDashboard.putBoolean("LL ready", Limelight.shootNow());
        SmartDashboard.putNumber("LL power" , Limelight.setPower());
        SmartDashboard.putNumber("LL distance" , Limelight.distanceFromTag());
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

    public static boolean getRightSide(){
        if(DriverStation.getAlliance().get() == Alliance.Red){
            return table.getEntry("tv").getDouble(0) == 5 || table.getEntry("tv").getDouble(0) == 15;
        }
        return table.getEntry("tv").getDouble(0) == 4 || table.getEntry("tv").getDouble(0) == 14;
    }

    /*
    *   @return returns horizontal distance from april tag in inches
    */
    public static double distanceFromTag(){
        // Angle between floor and april tag in radians
        double angle = Math.toRadians(Vision.LLAngle + getTy());
        // Difference between the height of barge tag and limelight
        double heightDiff = Vision.bargeTagHeight - Vision.LLHeight;
        return (heightDiff / Math.tan(angle)) +22; //inches
    }

    /*
    *   @return power to the drivetrain
    */
    public static double setPower(){
        if(!getTv()) return 0;
        if((distanceFromTag() - 206)/ 100.0 > 0.3) return 0.3;
        if((distanceFromTag() - 206)/ 100.0 < -0.3) return -0.3;
        return (distanceFromTag() - 206)/ 100.0;
    }

    public static boolean shootNow(){
        return Math.abs(distanceFromTag() - 206) < 3;
    }

    public static Pose2d getLimelightPose(){
        return LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight").pose;
    }
}