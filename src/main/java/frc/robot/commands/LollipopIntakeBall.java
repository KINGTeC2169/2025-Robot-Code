package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

//remove all the same things that we removed in IntakeBall
//We only use this command for autos (also we shouldn't have any autos with lollipop if mechanical does their fucking job)
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
        intake.setVoltageIntake(0.4*12);
        intake.setVoltageIndex(0.05*12);

    }

    @Override
    public void end(boolean interrupted){
        // no more suck

        intake.setVoltageIntake(0);
        intake.setVoltageIndex(0);
        intake.setIntakePos(IntakeConstants.restball);
        System.out.println(timer.get());
        timerStarted = false;

        intake.setVoltageIntake(0.05*12);
        intake.setVoltageIndex(0);
        //intake.setIntakePos(IntakeConstants.rest);
        
        
    }

    @Override
    public boolean isFinished(){
        return intake.distanceSensorCheckRange(0,4); //intake.getPosition() == intake.getSetPosition();
    }
}
