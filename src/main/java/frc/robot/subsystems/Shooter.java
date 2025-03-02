package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import java.util.Map;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {


    private VelocityVoltage motorVelocity = new VelocityVoltage(0);

    private TalonFX kraken; 
    private double testSpeed = 0;
    private double targetRPM = 0;
    private double timer;
    private boolean timerStart;



    public Shooter() {
        timerStart = false;
        
        kraken = new TalonFX(Constants.Ports.shooterMotor);


        var configs = new TalonFXConfiguration();
        configs.Slot0.kP = 0.25; //0.25
        kraken.getConfigurator().apply(configs, 0.05);
        kraken.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive), 0.05);

        //ShuffleboardLayout krak = tab.getLayout("Kraken", "List Layout").withPosition(0, 0).withSize(2, 2);
        //krak.addDouble("Top Motor RPM", () -> getRPM()).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("Max", 6000));
        


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

    public void setTargetRPM(double rpm){
        targetRPM = rpm;
    }

    public boolean isReady(){
        return getRPM() > targetRPM-50 && getRPM() < targetRPM+50;
    }

    public void setRPM(){
        testSpeed = targetRPM * (Math.PI * 0.00785);
        //System.out.println(testSpeed);
        motorVelocity.withVelocity(testSpeed);
        kraken.setControl(motorVelocity);
    }

    public void stopShooter(){
        kraken.set(0);
    }

    public void setMotorWithPID(){
        setRPM();
    }
    

    @Override
    public void periodic(){

        if(kraken.getSupplyCurrent().getValueAsDouble() > 15){
            timer++;
            timerStart = true;
        } 
        else if(timerStart){
            System.out.println(timer);
            timerStart = false;
            timer = 0;
        }

        SmartDashboard.putNumber("Top Motor RPM", getRPM());
        SmartDashboard.putBoolean("shoot ready",getRPM() > 5000);
        SmartDashboard.putNumber("shoot amps", kraken.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("shoot volts", kraken.getMotorVoltage().getValueAsDouble());
    }
}
