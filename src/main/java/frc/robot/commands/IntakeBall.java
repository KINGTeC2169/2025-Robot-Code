package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;

public class IntakeBall extends Command{
    private Intake intake;
    
    public IntakeBall(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.setIntakePos(IntakeConstants.grab); //Sets intake to grab position
        
    }

    @Override
    public void execute(){
        intake.setVoltageIntake(0.4*12); //runs intake to suck in the ball
        intake.setVoltageIndex(0.05*12); //runs indexer slightly backward to stop the ball from going into the shooter
    }

    @Override
    public void end(boolean interrupted){
        LED.setBlue(); //turns on the LED to show that the intake has a ball
        intake.setIntakePos(IntakeConstants.rest); //Puts intake in rest position
        intake.setVoltageIntake(0.05*12); //Keeps the intake motor running to keep the ball in the intake
        intake.setVoltageIndex(0); //Stops indexer
    }

    @Override
    public boolean isFinished(){
        return intake.distanceSensorCheckRange(0,4);
    }
}
