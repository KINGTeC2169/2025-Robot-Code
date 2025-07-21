package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {


    private VelocityVoltage motorVelocity = new VelocityVoltage(0);

    private TalonFX kraken; 
    private double targetRPM = 0; //RPM

    public Shooter() {     
        kraken = new TalonFX(Constants.Ports.shooterMotor);

        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = ShooterConstants.kS; // Add 0.1 V output to overcome static friction
        slot0Configs.kV = ShooterConstants.kV; // A velocity target of 1 rps results in 0.13 V output
        slot0Configs.kP = ShooterConstants.kP; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = ShooterConstants.kI; // no output for integrated error
        slot0Configs.kD = ShooterConstants.kD; // no output for error derivative

        kraken.getConfigurator().apply(slot0Configs); 
        kraken.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive), 0.05);
    }

    //Returns the TalonFX motor controller for the shooting motor
    public TalonFX getMotor() {
        return kraken;
    }

    //Returns the current of the shooting motor
    public double getCurrent(){
        return kraken.getSupplyCurrent().getValueAsDouble();
    }

    //Returns the voltage of the shooting motor
    public double getVoltage(){
        return kraken.getSupplyVoltage().getValueAsDouble();
    }

    //Returns the velocity of the shooting motor in RPM
    public double getRPM(){
        return 60 * kraken.getRotorVelocity().getValueAsDouble();
    }

    //Sets the targetRPM for the shooting motor. This is the RPM that the motor will try to reach.
    public void setTargetRPM(double rpm){
        targetRPM = rpm;
    }

    //Returns true if the shooting motor is within 1% of the targetRPM. This is used to check if the motor is ready to shoot.
    public boolean isReady(){
        return Math.abs(targetRPM) * 0.01 > Math.abs(targetRPM - getRPM());
    }

    //This is a PRIVATE method that sets the motor to the targetRPM. It is called in the periodic method.
    private void setRPM(){
        motorVelocity.withVelocity(targetRPM/60.0);
        kraken.setControl(motorVelocity);
    }

    //This is called every 20ms. It sets the motor to the targetRPM and updates the SmartDashboard with the current RPM, current, voltage, and if the motor is ready to shoot.
    @Override
    public void periodic(){

        setRPM();

        SmartDashboard.putNumber("Top Motor RPM", getRPM());
        SmartDashboard.putBoolean("shoot ready",isReady());
        SmartDashboard.putNumber("shoot amps", getCurrent());
        SmartDashboard.putNumber("shoot volts", getVoltage());
        
    }
}
