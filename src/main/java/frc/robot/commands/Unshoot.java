package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

//delete command or make it work (by adding intake running backwards and stuff)
public class Unshoot extends Command {
    
    private Shooter shooter;
    
    // Creates a new Unshoot command to run the shooter backwards to unjam the ball
    public Unshoot(Shooter shoot){
        shooter = shoot;
        addRequirements(shooter);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shooter.setTargetRPM(-1500); // set the target RPM to negative to run backwards
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.setTargetRPM(-1500); // set the target RPM to negative to run backwards
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) { 
        shooter.setTargetRPM(0);
    }
    
    
    @Override
    public boolean isFinished() {
        return shooter.getRPM() > -1300; //check if the RPM is below -1300 to end the command, This will give a short burst to put the ball back in the intake
    }
    

}

