package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

//delete command
public class uprightAdjustBall extends Command{
    private Intake intake;
    private Shooter shooter;
    private Timer timer;
    private double position;
    
    
    public uprightAdjustBall(Intake intake, Shooter shooter){
        this.intake = intake;
        addRequirements(intake);
        this.shooter = shooter;
        addRequirements(shooter);
        timer = new Timer();
        position = IntakeConstants.rest;
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        

    }

    @Override
    public void end(boolean interrupted){
        position = IntakeConstants.rest;
        System.out.println(position);
    }

    @Override
    public boolean isFinished(){
      
        return (intake.getPosition() > 0.1 && shooter.getCurrent() > 0.4) || intake.getPosition() >= IntakeConstants.rest;//intake.getDistance();
    }
}
