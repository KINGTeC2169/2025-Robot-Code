package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    private final VelocityVoltage motorVelocity = new VelocityVoltage(0);

    private TalonFX kraken; 

    private double targetRPM = 0;

    public Shooter() {
        
        kraken = new TalonFX(Constants.Ports.kraken);

        var configs = new Slot0Configs();
        configs.kP = 0.25;

        kraken.getConfigurator().apply(configs, 0.05);
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
        motorVelocity.withVelocity(targetRPM);
        kraken.setControl(motorVelocity);
    }

    public void stopShooter(){
        kraken.set(0);
    }




}