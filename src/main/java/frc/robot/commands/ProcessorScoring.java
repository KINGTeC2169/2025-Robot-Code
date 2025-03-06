package frc.robot.commands; 

import edu.wpi.first.wpilibj2.command.Command; 
import frc.robot.subsystems.Intake; 
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.DistanceSensor;

public class ProcessorScoring extends Command{
    private Intake intake;

    public ProcessorScoring(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }                    

    @Override
    public void initialize(){
        intake.setIntakePos(IntakeConstants.rest); 
        intake.shouldOuttake = true;
    }

    @Override
    //Runs intake backwards
    public void execute() { 
    }

    @Override
    //Stops intake
    public void end(boolean interupt) {
        intake.shouldOuttake = false;
        //intake.setIntakePos(IntakeConstants.grab); 
	}

    @Override
	public boolean isFinished() {
		return !intake.hasBall();
	}


}