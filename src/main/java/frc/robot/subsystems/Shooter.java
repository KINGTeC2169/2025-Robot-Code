package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {


    private VelocityVoltage motorVelocity = new VelocityVoltage(0);

    private TalonFX kraken; 
    private double testSpeed = 0;
    private double targetRPM = 0;
    private double timer;
    private boolean timerStart;

    //private PIDController shootPID;



    public Shooter() {
        timerStart = false;
        
        kraken = new TalonFX(Constants.Ports.shooterMotor);

        //shootPID = new PIDController(ShooterConstants.kP,ShooterConstants.kV,ShooterConstants.kS);
        // var configs = new TalonFXConfiguration();
        // configs.Slot0.kP = 0.05; //0.25
        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = ShooterConstants.kS; // Add 0.1 V output to overcome static friction
        slot0Configs.kV = ShooterConstants.kV; // A velocity target of 1 rps results in 0.13 V output
        slot0Configs.kP = ShooterConstants.kP; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = ShooterConstants.kI; // no output for integrated error
        slot0Configs.kD = ShooterConstants.kD; // no output for error derivative

        kraken.getConfigurator().apply(slot0Configs); 


        //kraken.getConfigurator().apply(configs, 0.05);
        kraken.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive), 0.05);

        //ShuffleboardLayout krak = tab.getLayout("Kraken", "List Layout").withPosition(0, 0).withSize(2, 2);
        //krak.addDouble("Top Motor RPM", () -> getRPM()).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("Max", 6000));
        


    }

    //voltage, also it should always go through targetRPM
    public void setPower(double power) {
        kraken.set(power);
    }

    //good
    public double getCurrent(){
        return kraken.getSupplyCurrent().getValueAsDouble();
    }

    //good
    public double getVoltage(){
        return kraken.getSupplyVoltage().getValueAsDouble();
    }

    //good
    public double getRPM(){
        return 60 * kraken.getRotorVelocity().getValueAsDouble();
    }

    //good
    public void setTargetRPM(double rpm){
        targetRPM = rpm;
    }

    //good
    public boolean isReady(){
        return getRPM() > targetRPM -(targetRPM * 0.01) && getRPM() < targetRPM + (targetRPM * 0.01);
    }

    //good
    public void setRPM(){
        //testSpeed = targetRPM * (Math.PI * 0.00785);
        testSpeed = targetRPM/60.0;
        //System.out.println(testSpeed);
        motorVelocity.withVelocity(testSpeed);
        kraken.setControl(motorVelocity);
    }

    //good
    public void stopShooter(){
        kraken.set(0);
    }

    //ok what the fuck
    public void setMotorWithPID(){
        setRPM();
    }
    

    //
    @Override
    public void periodic(){

        //only used for debugging, remove
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
        SmartDashboard.putBoolean("shoot ready",isReady());
        SmartDashboard.putNumber("shoot amps", kraken.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("shoot volts", kraken.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putBoolean("LL ready", Limelight.shootNow());
        SmartDashboard.putNumber("LL power" , Limelight.setPower());
        SmartDashboard.putNumber("LL distance" , Limelight.distanceFromTag());
        SmartDashboard.putNumber("tx", Limelight.getTx());
        //SmartDashboard.putData("shooter pid", ShooterConstants.kP);
    }
}
