package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DistanceSensor extends SubsystemBase{
    private Rev2mDistanceSensor distanceSensor;
    private ShuffleboardTab tab = Shuffleboard.getTab("Distance Sensor");

    public DistanceSensor(){
        distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
        distanceSensor.setDistanceUnits(Unit.kInches);
        distanceSensor.setAutomaticMode(true);
        distanceSensor.setEnabled(true);
        distanceSensor.setRangeProfile(RangeProfile.kDefault);

        tab.addDouble("Distance", () -> getDistance());
        tab.addDouble("Timestamp", () -> getTimeStamp());
        tab.addBoolean("Is Enabled", () -> isEnabled());
        tab.addBoolean("Has Valid Range", () -> hasValidRange());

    }

    public boolean hasValidRange(){
        return distanceSensor.isRangeValid();
    }

    public double getDistance(){
        return distanceSensor.getRange();
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
}
