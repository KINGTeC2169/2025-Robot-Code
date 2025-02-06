package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    private VelocityVoltage motorVelocity = new VelocityVoltage(0);

    private TalonFX kraken; 

    private double testSpeed = 0;

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
        //double testVelocity = (Math.PI * .00785 * targetRPM) / 60;
        //motorVelocity.withVelocity(testVelocity);
        //motorVelocity.withVelocity((2 * Math.PI) / targetRPM);
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
