package frc.robot.commands; 

import edu.wpi.first.wpilibj2.command.Command; 
import frc.robot.subsystems.Intake; 
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.DistanceSensor;

public class ProcessorScoring extends Command{
    private Intake intake;
    private final double rest = IntakeConstants.grab;

    public ProcessorScoring(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }                    

    @Override
    public void initialize(){
        intake.setIntakePos(0.1); 
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
		return !intake.hasBall();
        
	}


}