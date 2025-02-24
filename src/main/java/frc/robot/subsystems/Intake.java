package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
    private TalonFX pivotMotor;

    private DutyCycleEncoder encoder;
    private DistanceSensor distanceSensor;
    private PIDController pivotPID;
    
    private double setPosition;
    private double difference = 0; //between target and actual position

    private final double lowerLimit = IntakeConstants.rest; // needs to be fine tuned
    private final double upperLimit = IntakeConstants.grab; //limits for intake positions 
    private SimpleMotorFeedforward pivotForward; //check
    
     
    public Intake(){

        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kP = 0.01;

        if (RobotBase.isReal()) distanceSensor = new DistanceSensor();
        encoder = new DutyCycleEncoder(1,1,Constants.IntakeConstants.encoderOffset);
        
        intakeMotor = new TalonFX(Constants.Ports.intakeMotor);
        indexerMotor = new TalonFX(Constants.Ports.indexerMotor);
        pivotMotor = new TalonFX(Constants.Ports.pivotMotor);

        pivotPID = new PIDController(65,0.7,1);
        pivotForward = new SimpleMotorFeedforward(setPosition, lowerLimit, difference);

        intakeMotor.getConfigurator().apply(talonFXConfigs);
        intakeMotor.setNeutralMode(NeutralModeValue.Coast);
        indexerMotor.getConfigurator().apply(talonFXConfigs);
        indexerMotor.setNeutralMode(NeutralModeValue.Coast);
        pivotMotor.getConfigurator().apply(talonFXConfigs);
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    
    }

  /**Sets intake to suck in */
    public void sucker() {
        intakeMotor.set(0.3);
        indexerMotor.set(-0.35);
    }

    /**Runs intake backwards at 0.12 speed*/
    public void outTake() {
        intakeMotor.set(-0.3);
        indexerMotor.set(0.3);
    }

    /**Stops the intake. */
    public void stopTake(){
        intakeMotor.set(0);
        indexerMotor.set(0);
    }

    public boolean hasBall(){
    // Checks if ball is in intake to stop motor   
            if(RobotBase.isReal() && distanceSensor.ateBall()){
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
        //System.out.println("PID " + armPID.calculate(getPosition(), position) + " FORWARD " + armForward.calculate((position - 0.25)*6.28, 0));
        pivotMotor.setVoltage(pivotPID.calculate(getPosition(), position) + pivotForward.calculate((position - 0.25)*6.28, 0));
        difference = position - getPosition();
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
    public double getSpeed(){
        return intakeMotor.getVelocity().getValueAsDouble();
    }

    /**Returns the voltage of the intake motor. */
    public double getVoltage(){
        return intakeMotor.getSupplyVoltage().getValueAsDouble();
    }

    /**Returns the current of the intake motor. */
    public double getCurrent(){
        return intakeMotor.getSupplyCurrent().getValueAsDouble();
    }

    /**Returns true of the intake is on. */
    public boolean isOn(){
        return Math.abs(getSpeed()) > 0;
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
        double pos = (encoder.get());
         Math.abs(pos);
        return pos;
    }

    @Override   
    public void periodic() {
        SmartDashboard.putBoolean("Ball detected:", hasBall());
        SmartDashboard.putNumber("Intake Motor Speed", getSpeed());
        SmartDashboard.putNumber("IntakeM Voltage", getVoltage());
        SmartDashboard.putNumber("Confidence", getCurrent());
        SmartDashboard.putBoolean("Detected Color", isOn());
        SmartDashboard.putNumber("RPM", getRPM());
        SmartDashboard.putNumber("SetPosition", getSetPosition());
        SmartDashboard.putNumber("IntakePosition", getPosition());
    }
  
}
    

