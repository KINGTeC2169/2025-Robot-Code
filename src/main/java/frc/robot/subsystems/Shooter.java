package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.Slot0Configs;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    private VelocityVoltage motorVelocity = new VelocityVoltage(0);

    private TalonFX kraken; 

    private double testSpeed = 0;

    private double targetRPM = 0;

    private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");

    public Shooter() {
        
        kraken = new TalonFX(Constants.Ports.kraken);

        var configs = new TalonFXConfiguration();
        configs.Slot0.kP = 0.25;
        kraken.getConfigurator().apply(configs, 0.05);

        ShuffleboardLayout krak = tab.getLayout("Kraken", "List Layout").withPosition(0, 0).withSize(2, 2);
        krak.addDouble("Top Motor RPM", () -> getRPM()).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("Max", 6000));

    }

    public void setPower(double power) {
        kraken.set(power);
    }

    public double getCurrent(){
        return kraken.getSupplyCurrent().getValueAsDouble();
    }

    public double getVoltage(){
        return kraken.getSupplyVoltage().getValueAsDouble();
    }

    public double getRPM(){
        return 60 * kraken.getRotorVelocity().getValueAsDouble();
    }

    public void setRPM(double rpm){
        targetRPM = rpm;
        testSpeed = targetRPM * (Math.PI * 0.00785);
        System.out.println(testSpeed);
        motorVelocity.withVelocity(-testSpeed);
        kraken.setControl(motorVelocity);
        SmartDashboard.putNumber("RPM", getRPM());
    }

    public void stopShooter(){
        kraken.set(0);
    }

}
