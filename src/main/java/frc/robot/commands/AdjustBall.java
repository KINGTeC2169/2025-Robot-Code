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
        
    }

    @Override
    public void execute(){
        intake.shouldOuttake = false;
    }

    @Override
    public void end(boolean interrupted){
        // no more suck
        
        //intake.setIntakePos(IntakeConstants.restball);
        intake.setVoltageIndex(0);
        intake.setVoltageIntake(0);
        shooter.setPower(0);
        intake.shouldIntake = false;
        //intake.setIntakePos(IntakeConstants.rest);
        
        
    }

    @Override
    public boolean isFinished(){
      
        return intake.ateBall();//intake.getPosition() == intake.getSetPosition();
    }
}
