package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.ReefIntakeConstants;

public class ReefIntake extends SubsystemBase {
	private TalonFX reefPivotMotor;
    private SparkMax reefIntakeMotor;

    private SparkMaxConfig intakeConfig;

    private DutyCycleEncoder encoder;

    private PIDController pivotPID;
    private ArmFeedforward armFeedforward; 

    private final double upperLimit = ReefIntakeConstants.reefrest; // needs to be fine tuned
    private final double lowerLimit = ReefIntakeConstants.reefgrab; //limits for intake positions 
    
    private double setPosition;
    private double difference;

    //Reef Intake is 2.8 lbs
    //Bar is 20 inches long

    public ReefIntake(){
        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kP = 0.01;

        intakeConfig = new SparkMaxConfig();

        intakeConfig.idleMode(IdleMode.kBrake);

        intakeConfig.encoder.positionConversionFactor(1000);
        intakeConfig.encoder.velocityConversionFactor(1000);

        intakeConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1.0,0.0,0.0);

        encoder = new DutyCycleEncoder(1,1,Constants.ReefIntakeConstants.encoderOffset);
        
        reefPivotMotor = new TalonFX(Constants.Ports.reefPivotMotor);
        reefIntakeMotor = new SparkMax(Constants.Ports.reefIntakeMotor, MotorType.kBrushless);

        reefIntakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pivotPID = new PIDController(ReefIntakeConstants.kP, ReefIntakeConstants.kI,ReefIntakeConstants.kD);
        armFeedforward = new ArmFeedforward(ReefIntakeConstants.kS, ReefIntakeConstants.kG, ReefIntakeConstants.kV);
        
        reefPivotMotor.getConfigurator().apply(talonFXConfigs);
        reefPivotMotor.setNeutralMode(NeutralModeValue.Brake);
        // reefPivotMotor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), null, null);
        //reefPivotMotor.getConfigurator().apply(talonFXConfigs);
        //reefPivotMotor.setNeutralMode(NeutralModeValue.Brake);

        setIntakePos(ReefIntakeConstants.reefrest); //uncomment tis
    }

    

    //********************************************************************************************************************************* */

  /**Sets intake to suck in */
    public void sucker() {
        reefIntakeMotor.setVoltage(0.2*12);//0.3
       
    }

    /**Runs intake backwards at 0.12 speed*/
    public void outTake() {
        reefIntakeMotor.setVoltage(-2.4);
       
    }

    /**Stops the intake. */
    public void stopTake(){
        reefIntakeMotor.set(0);
        
    }   
    
    public void setVoltageIntake(double volts){
        reefIntakeMotor.setVoltage(volts);
        
    }

    public void setVoltagePivot(double volts){
        reefPivotMotor.setVoltage(volts);
        System.out.println("Volts " + volts);
    }

    public void setIntakePos(double position) {
        position = MathUtil.clamp(position, lowerLimit, upperLimit);
        
        setPosition = position;
        //reefPivotMotor.setVoltage(-pivotPID.calculate(getPosition(), position));
        System.out.println("PID " + pivotPID.calculate(getPosition(), position));
        System.out.println("FeedForward" + armFeedforward.calculate(getPosition() * 6.28 ,position * 6.28));//radians
        System.out.println("Both " + (pivotPID.calculate(getPosition(), position) + armFeedforward.calculate(getPosition(),position)));
    }

    /**Stops all motors */
    public void stopAll(){
        reefIntakeMotor.set(0);

        reefPivotMotor.set(0);

    }
    
    /**Returns the velocity of the intake motor. */
    public double getreefgrabSpeed(){
        return reefIntakeMotor.getEncoder().getVelocity();
    }

    /**Returns the voltage of the intake motor. */
    public double getreefgrabVoltage(){
        return reefIntakeMotor.getBusVoltage();
    }

    /**Returns the current of the reef intake motor. */
    public double getreefgrabCurrent(){
        return reefIntakeMotor.getOutputCurrent();
    }

    public double getPivotSpeed(){
        return reefPivotMotor.getVelocity().getValueAsDouble();
    }

    public double getPivotVoltage(){
        return reefPivotMotor.getMotorVoltage().getValueAsDouble();
    }

    public double getPivotCurrent(){
        return reefPivotMotor.getSupplyCurrent().getValueAsDouble();
    }

    /**Returns true of the intake is on. */
    public boolean isOn(){
        return Math.abs(getreefgrabSpeed()) > 0;
    }
        /**Returns the intake motor's rotor velocity in rotations per minute */
    public double getRPM(){
        return -(60 * reefIntakeMotor.getEncoder().getVelocity());
    }
    public double getSetPosition(){
        return setPosition;
    }

    /**Gets the position of the arm from the hex encoder */
    public double getPosition(){
        return encoder.get();
    }
    // checks if the reef intake is ready to intake based on the position its in.     
    public boolean isReady(){
        return difference < 0.0025 || (getSetPosition() == ReefIntakeConstants.reefgrab && getPosition() < ReefIntakeConstants.reefgrab);
    }

    @Override   
    public void periodic() {

        difference = Math.abs(setPosition - getPosition());

        // SmartDashboard.putBoolean("Ball Ate detected:", ateBall());
        // SmartDashboard.putBoolean("Ball detected:", hasBall());
        SmartDashboard.putNumber("reef Intake Motor Speed", getreefgrabSpeed());
        SmartDashboard.putNumber("reef IntakeM Voltage", getreefgrabVoltage());
        SmartDashboard.putBoolean("reef Detected Color", isOn());
        SmartDashboard.putNumber("reef RPM", getRPM());
        SmartDashboard.putNumber("reef SetPosition", getSetPosition());
        SmartDashboard.putNumber("reef IntakePosition", getPosition());
        SmartDashboard.putNumber("reef Pivot Speed", getPivotSpeed());
        SmartDashboard.putNumber("reef Pivot Voltage", getPivotVoltage());
        SmartDashboard.putNumber("reef Pivot Current", getPivotCurrent());
        SmartDashboard.putBoolean("reef Pivot Ready", isReady());
        SmartDashboard.putData("reef Pivot PID", pivotPID);

        

        setIntakePos(getSetPosition());

}
}
