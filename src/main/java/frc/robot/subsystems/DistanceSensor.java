package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//  import edu.wpi.first.cscore.CameraServerJNI.TelemetryKind;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DistanceSensor extends SubsystemBase{
    private Rev2mDistanceSensor distanceSensor;
    
    public DistanceSensor(){
        distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
        distanceSensor.setDistanceUnits(Unit.kInches);
        distanceSensor.setAutomaticMode(true);
        distanceSensor.setEnabled(true);
        distanceSensor.setRangeProfile(RangeProfile.kDefault);
    }
    public boolean hasBall(){
        if (distanceSensor.getRange() < 12 && distanceSensor.getRange() > 0) {//12
            return true;
        }
        return false;
    }

    public boolean ateBall(){
        if (distanceSensor.getRange() < 2.5 && distanceSensor.getRange() > 0) {//12
            return true;
        }
        return false;
    }

    public boolean adjustedBall(){
        if (distanceSensor.getRange() < 1.5 && distanceSensor.getRange() > 0) {//12
            return true;
        }
        return false;
    }

    public double getDistance(){
        if (distanceSensor.isRangeValid()) {
            return (distanceSensor.getRange());
        }else{
            return 0;
        }

             
    }

    public boolean isEnabled(){
        
        return distanceSensor.isEnabled();
    }

    public double getTimeStamp(){
        return distanceSensor.getTimestamp();
    }

    public void setEnabled(boolean x){
        distanceSensor.setEnabled(x);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("BallDistance:", getDistance());
        SmartDashboard.putNumber("Timestamp", getTimeStamp());
        SmartDashboard.putBoolean("Is Enabled", isEnabled());
        SmartDashboard.putBoolean("Is ball in?", ateBall());
    }
}
