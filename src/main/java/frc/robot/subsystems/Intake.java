package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;





public class Intake extends SubsystemBase {

    
  
  /** Creates a new Intake. */
    private TalonFX intakeMotor;
    private DutyCycleEncoder encoder;
    private DistanceSensor distanceSensor;
    private PIDController intakePID;
    
    private double setPosition;
    private double difference = 0; //between target and actual position

    private final double lowerLimit = IntakeConstants.rest;
    private final double upperLimit = IntakeConstants.net + 0.05; //limits for intake positions
    private ArmFeedforward intakeForward; //check
    

    private ShuffleboardTab tab = Shuffleboard.getTab("Intake");
    
     public Intake(){

         var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kP = 0;

        intakeMotor.getConfigurator().apply(talonFXConfigs);
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
        
        distanceSensor = new DistanceSensor();
        encoder = new DutyCycleEncoder(1,1,Constants.IntakeConstants.encoderOffset);
        
        intakeMotor = new TalonFX(Constants.Ports.intakeMotor);
        intakePID = new PIDController(65,0.7,1);
        intakeForward = new ArmFeedforward(setPosition, lowerLimit, difference);
        

    }


  /**Sets intake to suck in */
    public void Sucker() {
        intakeMotor.set(-0.4);
    }


    /**Runs intake backwards at 0.12 speed*/
    public void outTake() {
        intakeMotor.set(0.12);
    }

    /**Stops the intake. */
    public void stopTake(){
        intakeMotor.set(0);
    }

    public void hasBall(){
    // Checks if ball is in intake to stop motor   
            if(distanceSensor.hasValidRange()){
                stopTake();
                //LEDs.green();
            }else{
                Sucker();
               // LEDs.red(); 
            }
    }
    
    public void setVoltage(double volts){
        intakeMotor.setVoltage(volts);
    }


    public void setIntakePos(double position) {
        position = MathUtil.clamp(position, lowerLimit, upperLimit);
        setPosition = position;
        //System.out.println("PID " + armPID.calculate(getPosition(), position) + " FORWARD " + armForward.calculate((position - 0.25)*6.28, 0));
        intakeMotor.setVoltage(intakePID.calculate(getPosition(), position) + intakeForward.calculate((position - 0.25)*6.28, 0));
        difference = position - getPosition();
    }



    /**Stops the left and right motors */
    public void stop(){
        intakeMotor.set(0);
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
        double pos = (Constants.IntakeConstants.encoderOffset - encoder.get());
        if(pos < 0){
            pos += 1;
            Math.abs(pos);
        }
        return pos;
    }



  
  
}
    

