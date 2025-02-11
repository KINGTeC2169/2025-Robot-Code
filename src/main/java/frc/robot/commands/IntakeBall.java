package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeBall extends Command{
    private Intake intake;
    private boolean started;

    IntakeBall(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute(){
        // suck
        intake.sucker();
        // when motor get to certain speed its start time
        if(intake.getRPM() > 1750){
            started = true;
        } else if (intake.getRPM() < 1600 && started){
            // ???? what am i supposed to put here???
            // i dont even know my own code bruh
            isFinished();
        }
    }

    @Override
    public void end(boolean interrupted){
        // no more suck
        intake.stopTake();
    }

    @Override
    public boolean isFinished(){
        // idk how to do this *shrugging emoji*
        // is ball in intake? is it not in intake? 
        // who knows. not me. perhaps you? please know.
        return true;
    }
}
