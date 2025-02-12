package frc.robot.commands; 

import edu.wpi.first.wpilibj2.command.Command; 
import frc.robot.subsystems.Intake; 

public class processorScoring extends Command{
    private Intake intake;

    public processorScoring(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }                    

    @Override
    public void initialize(){
        intake.setIntakePos(0.27); //fine tune for actual value
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
	}

    @Override
	public boolean isFinished() {
		return false;
	}


}