package frc.robot.commands; 

import edu.wpi.first.wpilibj2.command.Command; 
import frc.robot.subsystems.Intake; 
import frc.robot.Constants.IntakeConstants;

public class ProcessorScoring extends Command{
    private Intake intake;

    public ProcessorScoring(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }                    

    @Override
    public void initialize(){
        intake.setIntakePos(IntakeConstants.rest); 
        intake.setVoltageIntake(-2.4);
        intake.setVoltageIndex(2.4);
    }

    @Override
    //Runs intake backwards
    public void execute() { 
    }

    @Override
    //Stops intake
    public void end(boolean interupt) {
        intake.setVoltageIntake(0);
        intake.setVoltageIndex(0);
        //intake.setIntakePos(IntakeConstants.grab); 
	}

    @Override
	public boolean isFinished() {
		return !intake.distanceSensorCheckRange(0,4);
	}


}