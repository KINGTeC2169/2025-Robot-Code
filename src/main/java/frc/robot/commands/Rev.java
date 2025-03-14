package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Rev extends Command {
    
    private Shooter shooter;
    private double rpm;
    
    public Rev(Shooter shoot, double rpm){
        shooter = shoot;
        addRequirements(shooter);
        this.rpm = rpm;
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shooter.setTargetRPM(rpm); // set the target RPM to the desired RPM to run the shooter at
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }
    
    
    @Override
    public boolean isFinished() {
        return shooter.isReady();
    }
    

}

