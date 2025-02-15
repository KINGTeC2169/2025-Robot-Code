package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter;

public class Rev extends Command {
    
    private Shooter shooter;
    private  CommandXboxController controller;
    
    public Rev(Shooter shoot, CommandXboxController controller) {
        shooter = shoot;
        this.controller = controller;
        addRequirements(shooter);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shooter.setRPM(6000 * controller.getLeftTriggerAxis());
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
        return false;
    }
    

}

