package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

public class Rev extends Command {
    
    private Shooter shooter;
    private Intake index;
    private Timer timer;
    
    public Rev(Shooter shoot, Intake index) {
        shooter = shoot;
        this.index = index;
        addRequirements(shooter);
        addRequirements(index);
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
        Timer.delay(0.75);
        shooter.setPower(0);
    }
    
    
    @Override
    public boolean isFinished() {
        return index.hasBall();
    }
    

}

