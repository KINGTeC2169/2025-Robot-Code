package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;


public class LollipopIntakeBall extends Command{
    private Intake intake;
    private Shooter shooter;
    private Timer timer;
    private boolean timerStarted;
    private int step;
    
    
    public LollipopIntakeBall(Intake intake, Shooter shooter){
        this.intake = intake;
        addRequirements(intake);
        this.shooter = shooter;
        addRequirements(shooter);
        timer = new Timer();
        timerStarted = false;
        step = 0;
    }

    @Override
    public void initialize(){
        intake.setIntakePos(IntakeConstants.restball);
        timer.start();
        timer.reset();
        timerStarted = false;
        
    }

    @Override
    public void execute(){
        // suck
        intake.shouldIntake = true;
        
        /* 
        if(intake.getGrabCurrent() > 9 && !timerStarted && timer.get() > 0.5){
            timer.reset();
            timerStarted = true;
            
        }*/
        System.out.println(timer.get());
        
        /* 
        intake.shouldIntake = true;

        if(intake.ateBall() && step == 0) step++;
        if(step == 1){
            intake.setIntakePos(IntakeConstants.restball);
            shooter.setPower(0);
            intake.shouldIntake = false;
            if(intake.isReady() && step == 1) step++;
        }
        if(step == 2){
            intake.shouldOuttakeAdjust = true;

        }
        */
        // if(intake.hasBall()){
        //     intake.setVoltageIndex(0);//-0.8
        //     intake.setVoltageIntake(0);//0.4
        //     shooter.setPower(0);
        //     if(!timerStarted){
        //         timerStarted = true;
        //         timer.reset();
        //         timer.start();
        //     }
        // } else {
        //     intake.sucker();
        //     shooter.setPower(0);//-0.25
        // }
        // System.out.println(timer.get());
        

    }

    @Override
    public void end(boolean interrupted){
        // no more suck

        intake.shouldIntake = false;
        intake.setIntakePos(IntakeConstants.restball);
        System.out.println(timer.get());
        timerStarted = false;

        intake.smallIntake = true;
        //intake.setIntakePos(IntakeConstants.rest);
        
        
    }

    @Override
    public boolean isFinished(){
        return intake.ateBall(); //intake.getPosition() == intake.getSetPosition();
    }
}
