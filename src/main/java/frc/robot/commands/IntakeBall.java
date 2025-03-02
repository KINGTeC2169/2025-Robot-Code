package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;


public class IntakeBall extends Command{
    private Intake intake;
    private Shooter shooter;
    private Timer timer;
    private boolean timerStarted;
    
    
    public IntakeBall(Intake intake, Shooter shooter){
        this.intake = intake;
        addRequirements(intake);
        this.shooter = shooter;
        addRequirements(shooter);
        timer = new Timer();
        timerStarted = false;

        
    }

    @Override
    public void initialize(){
        //intake.setIntakePos(IntakeConstants.grab);
        
    }

    @Override
    public void execute(){
        // suck
        
        if(intake.hasBall()){
            intake.setVoltageIndex(-0.8);
            intake.setVoltageIntake(0.4);
            shooter.setPower(0);
            if(!timerStarted){
                timerStarted = true;
                timer.reset();
                timer.start();
            }
        } else {
            intake.sucker();
            shooter.setPower(-0.25);
        }
        System.out.println(timer.get());
        

    }

    @Override
    public void end(boolean interrupted){
        // no more suck
        
        //intake.setIntakePos(IntakeConstants.rest);
        intake.setVoltageIndex(0);
        intake.setVoltageIntake(0);
        shooter.setPower(0);
        
    }

    @Override
    public boolean isFinished(){
      
        return timer.get() > 0.2 && timer.get() < 0.4;//intake.getPosition() == intake.getSetPosition();
    }
}
