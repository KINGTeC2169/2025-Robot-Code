package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class JustIntakeBallNoSensor extends Command{
    private Intake intake;
    private boolean lowRPM;
    private boolean highRPM;

    public JustIntakeBallNoSensor(Intake intake){
        this.intake = intake;
        addRequirements(intake);
        lowRPM = false;
        highRPM = false;
    }

    @Override
    public void initialize(){
        intake.setIntakePos(IntakeConstants.processor); 
    }

    @Override
    public void execute(){
        // suck
        intake.sucker();
        //if(intake.getRPM())

    }

    @Override
    public void end(boolean interrupted){
        // no more suck
        intake.stopTake();
        intake.setIntakePos(IntakeConstants.rest);
    }

    @Override
    public boolean isFinished(){
        // idk how to do this *shrugging emoji*
        // is ball in intake? is it not in intake? 
        // who knows. not me. perhaps you? please know.
        //i know

        // code works now (probably) *thumbs up*

        //khanh im from the future changing your code it doesnt work smhsmh
        return false;
    }
}
