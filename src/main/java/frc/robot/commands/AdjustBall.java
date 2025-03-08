package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;


public class AdjustBall extends Command{
    private Intake intake;
    private Shooter shooter;
    private Timer timer;
    
    
    public AdjustBall(Intake intake, Shooter shooter){
        this.intake = intake;
        addRequirements(intake);
        this.shooter = shooter;
        addRequirements(shooter);
        timer = new Timer();
        
    }

    @Override
    public void initialize(){
        intake.shouldOuttakeAdjust = true;
        timer.start();
        timer.reset();
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean interrupted){
        intake.shouldOuttakeAdjust = false;
        
        
    }

    @Override
    public boolean isFinished(){
      
        return intake.ateBall() && timer.get() > 0.04;//intake.getDistance();
    }
}
