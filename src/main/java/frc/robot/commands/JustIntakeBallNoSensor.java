package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

//remove this whole command too
//actually if we can make it work we can make as an alt button for operator (manual control)
public class JustIntakeBallNoSensor extends Command{
    private Intake intake;
    private boolean lowRPM;
    private boolean highRPM;
    private boolean started;
    private int counter;
    private Timer timer;
    private double t0;

    public JustIntakeBallNoSensor(Intake intake){
        this.intake = intake;
        addRequirements(intake);
        lowRPM = false;
        highRPM = false;
    }

    @Override
    public void initialize(){
        intake.setIntakePos(IntakeConstants.grab); 
    }

    @Override
    public void execute(){
        // suck
        intake.setVoltageIntake(0.4*12);
        intake.setVoltageIndex(0.05*12);
        if(intake.getIntakeVelocity() > 1700)counter++;
             if(counter > 10)started = true;
         
        if(started && intake.getIntakeVelocity() < 1550){
            intake.setVoltageIntake(0);
            intake.setVoltageIndex(0);
            started = false;
            if (!started){
                timer.start();
            }
        }
        
    }

    @Override
    public void end(boolean interrupted){
        // no more suck
        intake.setVoltageIntake(0);
        intake.setVoltageIndex(0);
        intake.setIntakePos(IntakeConstants.rest);
    }

    @Override
    public boolean isFinished(){
        return timer.get() > 1;
    }
}
