package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class IntakeBall extends Command{
    private Intake intake;
    
    
    public IntakeBall(Intake intake){
        this.intake = intake;
        addRequirements(intake);
        
    }

    @Override
    public void initialize(){
        //intake.setIntakePos(IntakeConstants.grab);
    }

    @Override
    public void execute(){
        // suck
        intake.sucker();

    }

    @Override
    public void end(boolean interrupted){
        // no more suck
        intake.stopTake();
        intake.setIntakePos(IntakeConstants.rest);
        
    }

    @Override
    public boolean isFinished(){
      
        return intake.hasBall();//intake.getPosition() == intake.getSetPosition();
    }
}
