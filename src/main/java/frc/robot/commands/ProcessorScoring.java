package frc.robot.commands; 

import edu.wpi.first.wpilibj2.command.Command; 
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.Constants.IntakeConstants;

public class ProcessorScoring extends Command{
    private Intake intake;

    public ProcessorScoring(Intake intake){
        this.intake = intake;  
        addRequirements(intake);
    }                    

    @Override
    public void initialize(){
        intake.setIntakePos(IntakeConstants.rest); // set the intake to rest position
        intake.setVoltageIntake(-1.5); // runs the indexer out of the intake 2.4
        intake.setVoltageIndex(1.5); // runs the intake out of the intake /2.4
    }

    @Override
    //Runs intake backwards
    public void execute() { 
    }

    @Override
    //Stops intake
    public void end(boolean interupt) {
        intake.setVoltageIntake(0);// stops the indexer
        intake.setVoltageIndex(0);// stops the intake
        LED.setRed();
	}

    @Override
	public boolean isFinished() {
		return !intake.distanceSensorCheckRange(0,14); // check if the intake doesnt have a ball to end the command
	}


}