package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;




public class Intake extends SubsystemBase {

    
  
  /** Creates a new Intake. */
    private TalonFX intakeMotor;
    private DutyCycleEncoder encoder;
    
    private DistanceSensor distanceSensor;
    private PIDController intakePID;
    private double setPosition;
    

    private ShuffleboardTab tab = Shuffleboard.getTab("Intake");
    
     public Intake(){

      
        
        distanceSensor = new DistanceSensor();
        encoder = new DutyCycleEncoder(1,1,Constants.IntakeConstants.encoderOffset);
        
        intakeMotor = new TalonFX(Constants.Ports.intakeMotor);
        intakePID = new PIDController(65,0.7,1);

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
            }else{
                Sucker();
            }
    }
    public void setPosition(){
            
    }
    






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


  
  
}
    

