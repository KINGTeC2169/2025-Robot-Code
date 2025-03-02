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

    private SparkMaxConfig pivotConfig;

    private DutyCycleEncoder encoder;
    private DistanceSensor distanceSensor;
    private PIDController pivotPID;
    private PIDController pivotPIDdown;
    
    private double setPosition;
    private double difference; //between target and actual position

    private final double upperLimit = IntakeConstants.rest; // needs to be fine tuned
    private final double lowerLimit = IntakeConstants.grab; //limits for intake positions 
    private SimpleMotorFeedforward pivotForward; //check

    public boolean shouldOuttake;
    public boolean shouldIntake;
    public boolean shouldIntakeOverride;
    
     
    public Intake(){

        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kP = 0.01;

        pivotConfig = new SparkMaxConfig();

        pivotConfig.idleMode(IdleMode.kBrake);

        pivotConfig.encoder.positionConversionFactor(1000);
        pivotConfig.encoder.velocityConversionFactor(1000);

        pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1.0,0.0,0.0);

        if (RobotBase.isReal()) distanceSensor = new DistanceSensor();
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

        //setIntakePos(IntakeConstants.rest); uncomment tis
        shouldOuttake = false;
        shouldIntake = false;
        shouldIntakeOverride = false;
    
    }

  /**Sets intake to suck in */
    public void sucker() {
        intakeMotor.setVoltage(0.2*12);//0.3
        indexerMotor.setVoltage(-0.2*12);//0.35
    }

    /**Runs intake backwards at 0.12 speed*/
    public void outTake() {
        intakeMotor.setVoltage(-2);
        indexerMotor.setVoltage(2);
    }

    /**Stops the intake. */
    public void stopTake(){
        intakeMotor.set(0);
        indexerMotor.set(0);
    }

    public boolean ateBall(){
    // Checks if ball is in intake to stop motor   
            if(RobotBase.isReal() && distanceSensor.ateBall()){
                //LEDs.green();
                return true; // Ball detected
            }else{
               // LEDs.red(); 
                return false; // No ball detected
            }
    }

    public boolean hasBall(){
        if(RobotBase.isReal() && distanceSensor.hasBall()){
            //LEDs.green();
            return true; // Ball detected
        }else{
           // LEDs.red(); 
            return false; // No ball detected
        }
    }
    
    public void setVoltageIntake(double volts){
        intakeMotor.setVoltage(volts);
    }

    public void setVoltageIndex(double volts){
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

    public void setMotorDistanceSensor(){
        if(shouldOuttake){
            outTake();
        } else if((shouldIntake && !distanceSensor.ateBall() )||shouldIntakeOverride){
            sucker();
        } else {
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

        if(!(getSetPosition() == IntakeConstants.grab)){
            setIntakePos(getSetPosition());
        } else {
            if(isReady() || ateBall()) setVoltagePivot(0);
            setIntakePos(Constants.IntakeConstants.grab);

        }

        
        

        //if (!encoder.isConnected()) Elastic.sendNotification(new Notification().withLevel(NotificationLevel.WARNING)
                                                                            //    .withTitle("Warning")
                                                                            //    .withDescription("Intake Hex Encoder Disconnected")
                                                                            //    .withDisplaySeconds(5));

    }
  
}
    

