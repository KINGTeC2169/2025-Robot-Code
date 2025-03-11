package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;


public class IntakeBall extends Command{
    private Intake intake;
    private Shooter shooter;//remove
    private Timer timer;//remove
    private boolean timerStarted;//remove
    private int step;//remove
    
    
    public IntakeBall(Intake intake, Shooter shooter){
        this.intake = intake;
        addRequirements(intake);
        this.shooter = shooter;//remove
        addRequirements(shooter);//remove
        timer = new Timer(); //remove
        timerStarted = false; //remove
        step = 0; //remove
    }

    @Override
    public void initialize(){
        intake.setIntakePos(IntakeConstants.grab);
        timer.start();//remove
        timer.reset();//remove
        timerStarted = false;//remove
        
    }

    @Override
    public void execute(){
        intake.shouldIntake = true; //remove
        System.out.println(timer.get());//remove
    }

    @Override
    public void end(boolean interrupted){
        intake.shouldIntake = false; //remove
        intake.setIntakePos(IntakeConstants.rest);
        System.out.println(timer.get()); //remove
        timerStarted = false; //remove
        intake.smallIntake = true; //remove
    }

    @Override
    public boolean isFinished(){
        return intake.ateBall();
    }
}
