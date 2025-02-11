package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeBall extends Command{
    private Intake intake;

    IntakeBall(Intake i){
        intake = i;
        addRequirements(intake);
    }

    public void execute(){
        // suck
        intake.sucker();
    }

    public void end(){
        // no more suck
        intake.stopTake();
    }

    public boolean isFinished(){
        // idk how to do this *shrugging emoji*
        // is ball in intake? is it not in intake? 
        // who knows. not me. perhaps you? please know.
        return false;
    }
}
