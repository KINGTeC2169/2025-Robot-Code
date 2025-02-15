package frc.robot.commands; 

import java.util.concurrent.Flow.Processor;

import edu.wpi.first.wpilibj2.command.Command; 
import frc.robot.subsystems.Intake; 
import frc.robot.Constants.IntakeConstants;

public class ProcessorScoring extends Command{
    private Intake intake;
    private final double processor = IntakeConstants.processor;
    private final double rest = IntakeConstants.rest;

    public ProcessorScoring(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }                    

    @Override
    public void initialize(){
        intake.setIntakePos(processor); 
    }

    @Override
    //Runs intake backwards
    public void execute() { 
        intake.outTake();
    }

    @Override
    //Stops intake
    public void end(boolean interupt) {
        intake.stopTake();
        intake.setIntakePos(rest); 
	}

    @Override
	public boolean isFinished() {
		return false;
	}


}