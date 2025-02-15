package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Rev extends Command {
    
    private Shooter shooter;
    private Timer timer;
    private double start;
    
    public Rev(Shooter shoot) {
        shooter = shoot;
        addRequirements(shooter);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shooter.setRPM(6000);
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.setRPM(6000);
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        /*
        fix with prob systmecurrnentmillisss
        timer.wait(500);
        */
        shooter.setPower(0);
    }
    
    
    @Override
    public boolean isFinished() {
        return Intake.hasBall();
    }
    

}

