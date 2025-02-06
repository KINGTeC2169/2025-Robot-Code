package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;


import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Modes;

public class Intake extends SubsystemBase {
  
  /** Creates a new Intake. */
  private TalonFX intakeMotor;

  private ShuffleboardTab tab = Shuffleboard.getTab("Intake");


  public int getMode(){
    return Modes.intakeMode;
  }

  public void cycleMode(){
      Modes.intakeMode++;
      Modes.intakeMode%=3;
  }

  /**Sets intake to suck in */
  public void Sucker() {
      intakeMotor.set(-0.4);
  }

  /**Returns the intake motor's rotor velocity in rotations per minute */
  public double getRPM(){
      return -(60 * intakeMotor.getRotorVelocity().getValueAsDouble());
  }

  /**Runs intake backwards at 0.12 speed*/
  public void outTake() {
      intakeMotor.set(0.12);
  }

  /**Stops the intake. */
  public void stopTake(){
      intakeMotor.set(0);
  }

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


  
  
}
    

