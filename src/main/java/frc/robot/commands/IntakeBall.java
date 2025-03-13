package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;


public class IntakeBall extends Command{
    private Intake intake;
    
    public IntakeBall(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.setIntakePos(IntakeConstants.grab);
        
    }

    @Override
    public void execute(){
        intake.setVoltageIntake(0.4*12);
        intake.setVoltageIndex(0.05*12);
    }

    @Override
    public void end(boolean interrupted){
        intake.setIntakePos(IntakeConstants.rest);
        intake.setVoltageIntake(0.05*12);
        intake.setVoltageIndex(0);
    }

    @Override
    public boolean isFinished(){
        return intake.distanceSensorCheckRange(0,4);
    }
}
