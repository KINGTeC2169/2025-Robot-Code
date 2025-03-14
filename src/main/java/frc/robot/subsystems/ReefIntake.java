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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.ReefIntakeConstants;

public class ReefIntake extends SubsystemBase {
	private SparkMax reefPivotMotor;
    private TalonFX reefIntakeMotor;

    private SparkMaxConfig pivotConfig;

    private DutyCycleEncoder encoder;

    private PIDController pivotPID;
    private PIDController pivotPIDdown;

    private final double upperLimit = ReefIntakeConstants.reefrest; // needs to be fine tuned
    private final double lowerLimit = ReefIntakeConstants.reefgrab; //limits for intake positions 
    
    private double setPosition;
    private double difference;

    public boolean shouldOuttake;
    public boolean shouldOuttakeAdjust;
    public boolean shouldIntake;
    public boolean shouldIntakeOverride;

    private boolean started = false;

    public ReefIntake(){
        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kP = 0.01;

        pivotConfig = new SparkMaxConfig();

        pivotConfig.idleMode(IdleMode.kBrake);

        pivotConfig.encoder.positionConversionFactor(1000);
        pivotConfig.encoder.velocityConversionFactor(1000);

        pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1.0,0.0,0.0);

        encoder = new DutyCycleEncoder(1,1,Constants.ReefIntakeConstants.encoderOffset);
        
        reefIntakeMotor = new TalonFX(Constants.Ports.reefIntakeMotor);
        reefPivotMotor = new SparkMax(Constants.Ports.reefPivotMotor, MotorType.kBrushless);

        reefPivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pivotPID = new PIDController(ReefIntakeConstants.kP, ReefIntakeConstants.kI,ReefIntakeConstants.kD);
        
        pivotPIDdown = new PIDController(ReefIntakeConstants.kPdown, ReefIntakeConstants.kIdown,ReefIntakeConstants.kDdown);

        reefIntakeMotor.getConfigurator().apply(talonFXConfigs);
        reefIntakeMotor.setNeutralMode(NeutralModeValue.Coast);
        // reefPivotMotor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), null, null);
        //reefPivotMotor.getConfigurator().apply(talonFXConfigs);
        //reefPivotMotor.setNeutralMode(NeutralModeValue.Brake);

        setIntakePos(ReefIntakeConstants.reefrestball); //uncomment tis
        shouldOuttake = false;
        shouldIntake = false;
        shouldIntakeOverride = false;
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
    

    public boolean reefAteBall(){
        if (getRPM() > 1700) {
            started = true; 
            
        }
        if(started && getRPM() < 1550){
            stopTake();
            started = false;
            return true;
            
        }
        return false;
    }

    

    

    public boolean ateBall(){
    // Checks if ball is in intake to stop motor   
            if(RobotBase.isReal() && reefAteBall()){
                //LEDs.green();
                return true; // Ball detected
            }else{
               // LEDs.red(); 
                return false; // No ball detected
            }
    }

   
    
    
    private void setVoltageIntake(double volts){
        reefIntakeMotor.setVoltage(volts);
    }

    public void setVoltagePivot(double volts){
        reefPivotMotor.setVoltage(volts);
    }

    public void setIntakePos(double position) {
        position = MathUtil.clamp(position, lowerLimit, upperLimit);
        setPosition = position;
        if(!(getSetPosition() == ReefIntakeConstants.reefgrab)) reefPivotMotor.setVoltage(-pivotPID.calculate(getPosition(), position)); //+ pivotForward.calculate(position));
        else {
            if(!isReady()) reefPivotMotor.setVoltage(-pivotPIDdown.calculate(getPosition(), position));
            else reefPivotMotor.setVoltage(0);
        }
    }

    /**Stops all motors */
    public void stopAll(){
        reefIntakeMotor.set(0);
        
        reefPivotMotor.set(0);

    }

    //gets and misc status givers

    //********************************************************************************************************************************* */
    /**Returns the velocity of the intake motor. */
    public double getreefgrabSpeed(){
        return reefIntakeMotor.getVelocity().getValueAsDouble();
    }

    /**Returns the voltage of the intake motor. */
    public double getreefgrabVoltage(){
        return reefIntakeMotor.getSupplyVoltage().getValueAsDouble();
    }

    /**Returns the current of the reef intake motor. */
    public double getreefgrabCurrent(){
        return reefIntakeMotor.getSupplyCurrent().getValueAsDouble();
    }

    public double getPivotSpeed(){
        return reefPivotMotor.get();
    }

    public double getPivotVoltage(){
        return reefPivotMotor.getBusVoltage();
    }

    public double getPivotCurrent(){
        return reefPivotMotor.getOutputCurrent();
    }

    /**Returns true of the intake is on. */
    public boolean isOn(){
        return Math.abs(getreefgrabSpeed()) > 0;
    }
        /**Returns the intake motor's rotor velocity in rotations per minute */
    public double getRPM(){
        return -(60 * reefIntakeMotor.getRotorVelocity().getValueAsDouble());
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

    // This method runs periodically every 5ms
    public void setMotorDistanceSensor(){
        if(shouldOuttake || shouldOuttakeAdjust){
            outTake();
        } else if((shouldIntake && !reefAteBall()) || shouldIntakeOverride){
            sucker();
        } else {
            stopTake();
        }
    }

    @Override   
    public void periodic() {

        difference = Math.abs(setPosition - getPosition());

        // SmartDashboard.putBoolean("Ball Ate detected:", ateBall());
        // SmartDashboard.putBoolean("Ball detected:", hasBall());
        SmartDashboard.putNumber("Intake Motor Speed", getreefgrabSpeed());
        SmartDashboard.putNumber("IntakeM Voltage", getreefgrabVoltage());
        SmartDashboard.putBoolean("Detected Color", isOn());
        SmartDashboard.putNumber("RPM", getRPM());
        SmartDashboard.putNumber("SetPosition", getSetPosition());
        SmartDashboard.putNumber("IntakePosition", getPosition());
        SmartDashboard.putNumber("Pivot Speed", getPivotSpeed());
        SmartDashboard.putNumber("Pivot Voltage", getPivotVoltage());
        SmartDashboard.putNumber("Pivot Current", getPivotCurrent());
        SmartDashboard.putBoolean("Pivot Ready", isReady());
        SmartDashboard.putData("Pivot PID", pivotPID);
        SmartDashboard.putData("Pivot PID down", pivotPIDdown);

        

        if(!(getSetPosition() == ReefIntakeConstants.reefgrab)){
            setIntakePos(getSetPosition());
        } else {
            if(isReady() || ateBall()) setVoltagePivot(0);
            setIntakePos(Constants.ReefIntakeConstants.reefgrab);

        }

}
}
