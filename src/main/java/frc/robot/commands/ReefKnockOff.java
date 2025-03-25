package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ReefIntakeConstants;
import frc.robot.subsystems.ReefIntake;



public class ReefKnockOff extends Command{
    private ReefIntake reefIntake;
    
    public ReefKnockOff(ReefIntake reefIntake){
        this.reefIntake = reefIntake;
        addRequirements(reefIntake);
    }

    @Override
    public void initialize(){
        reefIntake.setIntakePos(ReefIntakeConstants.reefGrab); //Sets reefIntake to grab position       
    }

    @Override
    public void execute(){
        reefIntake.setVoltageIntake(2); //runs reefIntake to suck in the ball
    }    
    @Override
    public void end(boolean interrupted){
        reefIntake.setIntakePos(IntakeConstants.rest); //Puts reefIntake in rest position
        reefIntake.setVoltageIntake(0); //Keeps the reefIntake motor running to keep the ball in thereefIntake
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
