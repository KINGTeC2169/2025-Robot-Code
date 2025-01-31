package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import java.util.Map;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    
    private TalonFX kraken;

    private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

    public Shooter() {
        
        kraken = new TalonFX(Constants.Ports.kraken);

        var configs = new Slot0Configs();
        configs.kP = 0.25;

        kraken.getConfigurator().apply(configs, 0.05);

        /*
        possible shuffleboard code?
        
        ShuffleboardLayout kraken = tab.getLayout("Motor", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4);
        kraken.addDouble("Motor RPM", () -> getRPM()[0]).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("Max", 4000));
        kraken.addDouble("Motor Voltage", () -> getVoltage()[0]).withWidget(BuiltInWidgets.kVoltageView).withProperties(Map.of("Max", 12));
        kraken.addDouble("Motor Current", () -> getCurrent()[0]).withWidget(BuiltInWidgets.kDial);
        */
    }

    public void setPower(double power) {
        kraken.set(power);
    }



   
    
}
