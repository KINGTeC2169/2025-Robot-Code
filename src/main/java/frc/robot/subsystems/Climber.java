package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;


public class Climber extends SubsystemBase{
    private TalonFX climberMotor;
    private DutyCycleEncoder climberEncoder;
    private PIDController climberPID;
    public Climber() {
    
        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kP = 0.01;
        
        climberEncoder = new DutyCycleEncoder(2,1,Constants.ClimberConstants.encoderOffset);

        climberMotor = new TalonFX(Constants.Ports.climberMotor);

        climberPID = new PIDController(ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD);

        climberMotor.getConfigurator().apply(talonFXConfigs);
        climberMotor.setNeutralMode(NeutralModeValue.Brake);  
    }

    public double getPosition(){
        return climberEncoder.get();
    }

    public double getClimberVelocity(){
        return climberMotor.getVelocity().getValueAsDouble();
    }

    public double getClimberVolts(){
        return climberMotor.getSupplyVoltage().getValueAsDouble();
    }
    
    public double getClimberCurent(){
        return climberMotor.getSupplyCurrent().getValueAsDouble();
    }

    




      
}