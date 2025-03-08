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
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  /** Creates a new Intake. */
    private TalonFX intakeMotor;
    private TalonFX indexerMotor;
    private SparkMax pivotMotor;

    private Rev2mDistanceSensor distanceSensor;

    private SparkMaxConfig pivotConfig;

    private DutyCycleEncoder encoder;
    private PIDController pivotPID;
    private PIDController pivotPIDdown;
    
    private double setPosition;
    private double difference; //between target and actual position

    private final double upperLimit = IntakeConstants.rest; // needs to be fine tuned
    private final double lowerLimit = IntakeConstants.grab; //limits for intake positions 
    private SimpleMotorFeedforward pivotForward; //check

    public boolean shouldOuttake;
    public boolean shouldOuttakeAdjust;
    public boolean shouldIntake;
    public boolean shouldIntakeOverride;
    
    private double latestDistance;
    
     
    public Intake(){

        if (RobotBase.isReal()){
            distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
            distanceSensor.setDistanceUnits(Unit.kInches);
            distanceSensor.setAutomaticMode(true);
            distanceSensor.setEnabled(true);
            distanceSensor.setRangeProfile(RangeProfile.kDefault);
        }

        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kP = 0.01;

        pivotConfig = new SparkMaxConfig();

        pivotConfig.idleMode(IdleMode.kBrake);

        pivotConfig.encoder.positionConversionFactor(1000);
        pivotConfig.encoder.velocityConversionFactor(1000);

        pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1.0,0.0,0.0);

        encoder = new DutyCycleEncoder(1,1,Constants.IntakeConstants.encoderOffset);
        
        intakeMotor = new TalonFX(Constants.Ports.intakeMotor);
        indexerMotor = new TalonFX(Constants.Ports.indexerMotor);
        //pivotMotor = new TalonFX(Constants.Ports.pivotMotor);
        pivotMotor = new SparkMax(Constants.Ports.pivotMotor, MotorType.kBrushless);

        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pivotPID = new PIDController(IntakeConstants.kP, IntakeConstants.kI,IntakeConstants.kD);
        pivotForward = new SimpleMotorFeedforward(IntakeConstants.kS, IntakeConstants.kV, IntakeConstants.kA);

        pivotPIDdown = new PIDController(IntakeConstants.kPdown, IntakeConstants.kIdown,IntakeConstants.kDdown);

        intakeMotor.getConfigurator().apply(talonFXConfigs);
        intakeMotor.setNeutralMode(NeutralModeValue.Coast);
        indexerMotor.getConfigurator().apply(talonFXConfigs);
        indexerMotor.setNeutralMode(NeutralModeValue.Coast);
        // pivotMotor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), null, null);
        //pivotMotor.getConfigurator().apply(talonFXConfigs);
        //pivotMotor.setNeutralMode(NeutralModeValue.Brake);

        setIntakePos(IntakeConstants.restball); //uncomment tis
        shouldOuttake = false;
        shouldIntake = false;
        shouldIntakeOverride = false;
    
    }

    public boolean distanceSensorHasBall(){
        if (latestDistance < 15 && latestDistance > 0) {//12
            return true;
        }
        return false;
    }

    public boolean distanceSensorAteBall(){
        if (latestDistance < 4 && latestDistance > 0) {//12
            return true;
        }
        return false;
    }

    public boolean distanceSensorAdjustedBall(){
        if (latestDistance > 8.5) {//12
            return true;
        }
        return false;
    }

    public double distanceSensorGetDistance(){
        if (distanceSensor.isRangeValid()) {
            return (distanceSensor.getRange());
        }else{
            return 0;
        }         
    }

    public boolean distanceSensorIsEnabled(){
        return distanceSensor.isEnabled();
    }

    public double distanceSensorGetTimeStamp(){
        return distanceSensor.getTimestamp();
    }

    public void distanceSensorSetEnabled(boolean x){
        distanceSensor.setEnabled(x);
    }

  /**Sets intake to suck in */
    public void sucker() {
        intakeMotor.setVoltage(0.4*12);//0.2
        //indexerMotor.setVoltage(-0.2*12);//0.35
    }
    public void supersucker() {
        intakeMotor.setVoltage(0.7*12);//0.3
        indexerMotor.setVoltage(-0.7*12);//0.35
    }

    /**Runs intake backwards at 0.12 speed*/
    public void outTake() {
        intakeMotor.setVoltage(-2.4);
        indexerMotor.setVoltage(2.4);
    }

    /**Stops the intake. */
    public void stopTake(){
        intakeMotor.set(0);
        indexerMotor.set(0);
    }

    public boolean adjustBall(){
        // Checks if ball is in intake to stop motor   
                if(RobotBase.isReal() && distanceSensorAdjustedBall()){
                    //LEDs.green();
                    return true; // Ball detected
                }else{
                   // LEDs.red(); 
                    return false; // No ball detected
                }
        }

    public boolean ateBall(){
    // Checks if ball is in intake to stop motor   
            if(RobotBase.isReal() && distanceSensorAteBall()){
                //LEDs.green();
                return true; // Ball detected
            }else{
               // LEDs.red(); 
                return false; // No ball detected
            }
    }

    public boolean hasBall(){
        if(RobotBase.isReal() && distanceSensorHasBall()){
            //LEDs.green();
            return true; // Ball detected
        }else{
           // LEDs.red(); 
            return false; // No ball detected
        }
    }
    
    private void setVoltageIntake(double volts){
        intakeMotor.setVoltage(volts);
    }

    private void setVoltageIndex(double volts){
        indexerMotor.setVoltage(-volts);
    }
    public void setVoltagePivot(double volts){
        pivotMotor.setVoltage(volts);
    }

    public void setIntakePos(double position) {
        position = MathUtil.clamp(position, lowerLimit, upperLimit);
        setPosition = position;
        if(!(getSetPosition() == IntakeConstants.grab)) pivotMotor.setVoltage(-pivotPID.calculate(getPosition(), position)); //+ pivotForward.calculate(position));
        else {
            if(!isReady()) pivotMotor.setVoltage(-pivotPIDdown.calculate(getPosition(), position));
            else pivotMotor.setVoltage(0);
        }
    }

    /**Stops all motors */
    public void stopAll(){
        intakeMotor.set(0);
        indexerMotor.set(0);
        pivotMotor.set(0);

    }

    //gets and misc status givers

    //********************************************************************************************************************************* */
    /**Returns the velocity of the intake motor. */
    public double getGrabSpeed(){
        return intakeMotor.getVelocity().getValueAsDouble();
    }

    /**Returns the voltage of the intake motor. */
    public double getGrabVoltage(){
        return intakeMotor.getSupplyVoltage().getValueAsDouble();
    }

    /**Returns the current of the intake motor. */
    public double getGrabCurrent(){
        return intakeMotor.getSupplyCurrent().getValueAsDouble();
    }

    public double getPivotSpeed(){
        return pivotMotor.get();
    }

    public double getPivotVoltage(){
        return pivotMotor.getBusVoltage();
    }

    public double getPivotCurrent(){
        return pivotMotor.getOutputCurrent();
    }

    /**Returns true of the intake is on. */
    public boolean isOn(){
        return Math.abs(getGrabSpeed()) > 0;
    }
        /**Returns the intake motor's rotor velocity in rotations per minute */
    public double getRPM(){
        return -(60 * intakeMotor.getRotorVelocity().getValueAsDouble());
    }
    public double getSetPosition(){
        return setPosition;
    }

    /**Gets the position of the arm from the hex encoder */
    public double getPosition(){
        return encoder.get();
    }

    public boolean isReady(){
        return difference < 0.0025 || (getSetPosition() == IntakeConstants.grab && getPosition() < IntakeConstants.grab);
    }

    // This method runs periodically every 5ms
    public void setMotorDistanceSensor(){

        if(shouldOuttake || shouldOuttakeAdjust){
            outTake();
        } else if((shouldIntake )){//&& !distanceSensorAteBall())){
            sucker();
        } else if (shouldIntakeOverride) {
            supersucker();
        }else {
            stopTake();
        }
    }

    @Override   
    public void periodic() {

        difference = Math.abs(setPosition - getPosition());

        SmartDashboard.putBoolean("Ball Ate detected:", ateBall());
        SmartDashboard.putBoolean("Ball detected:", hasBall());
        SmartDashboard.putNumber("Intake Motor Speed", getGrabSpeed());
        SmartDashboard.putNumber("IntakeM Voltage", getGrabVoltage());
        SmartDashboard.putNumber("IntakeM Current", getGrabCurrent());
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

        SmartDashboard.putNumber("BallDistance:", latestDistance);

        if(!(getSetPosition() == IntakeConstants.grab)){
            setIntakePos(getSetPosition());
        } else {
            if(isReady() || ateBall()) setVoltagePivot(0);
            setIntakePos(Constants.IntakeConstants.grab);

        }
        latestDistance = distanceSensor.getRange();

        
        

        //if (!encoder.isConnected()) Elastic.sendNotification(new Notification().withLevel(NotificationLevel.WARNING)
                                                                            //    .withTitle("Warning")
                                                                            //    .withDescription("Intake Hex Encoder Disconnected")
                                                                            //    .withDisplaySeconds(5));

    }
  
}
    

