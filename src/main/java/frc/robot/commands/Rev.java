package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Rev extends Command {
    
    private Shooter shooter;
    private double rpm;
    
    public Rev(Shooter shoot, double rpm){//, CommandXboxController controller) {
        shooter = shoot;
        addRequirements(shooter);
        this.rpm = rpm;
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shooter.setTargetRPM(rpm); //* controller.getLeftTriggerAxis());
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //Timer.delay(0.75);
        //shooter.setPower(0);    
        
        
    }
    
    
    @Override
    public boolean isFinished() {
        return shooter.isReady();
    }
    

}

