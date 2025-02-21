package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import java.util.Map;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
    private PIDController pid = new PIDController(0.25, 0, 0);
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.25, 0.25);



    public Shooter() {
        
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

    public void vroom(double target){
        kraken.setVoltage(pid.calculate(kraken.getRotorVelocity().getValueAsDouble() * (0.00785 / 2), target) + feedforward.calculate(target));
    }

    public void setRPM(double rpm){
        targetRPM = rpm;

        testSpeed = targetRPM * (Math.PI * 0.00785);
        //System.out.println(testSpeed);
        motorVelocity.withVelocity(testSpeed);
        kraken.setControl(motorVelocity);
    }

    public void stopShooter(){
        kraken.set(0);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Top Motor RPM", getRPM());
    }
}

