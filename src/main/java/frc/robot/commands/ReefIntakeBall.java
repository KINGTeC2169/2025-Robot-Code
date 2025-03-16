package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ReefIntakeConstants;
import frc.robot.subsystems.ReefIntake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;



public class ReefIntakeBall extends Command{
    private ReefIntake reefIntake;
    private Shooter shooter;
    private Intake intake;
    
    public ReefIntakeBall(ReefIntake reefIntake, Shooter shooter, Intake intake){
        this.reefIntake = reefIntake;
        addRequirements(reefIntake);
        this.shooter = shooter;
        addRequirements(shooter);
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        reefIntake.setIntakePos(ReefIntakeConstants.reefgrab); //Sets reefIntake to grab position
        intake.setIntakePos(IntakeConstants.restball); //Puts intake in rest position
        
    }

    @Override
    public void execute(){
        reefIntake.setVoltageIntake(0.4*12); //runs reefIntake to suck in the ball
        shooter.setTargetRPM(-1000); //Sets shooter to run at -1000 RPM to suck in the ball and put it in the intake
        intake.setVoltageIntake(0.05*12); //Keeps the intake motor running to keep the ball in the intake
        intake.setVoltageIndex(3.6); //Stops indexer
        
    }    
    @Override
    public void end(boolean interrupted){
        reefIntake.setIntakePos(IntakeConstants.rest); //Puts reefIntake in rest position
        intake.setIntakePos(IntakeConstants.restball); //Pusts intake in rest position
        reefIntake.setVoltageIntake(0.05*12); //Keeps the reefIntake motor running to keep the ball in thereefIntake
        shooter.setTargetRPM(0); //Stops shooter
        intake.setVoltageIndex(0); //Stops indexer
    }

    @Override
    public boolean isFinished(){
        return intake.distanceSensorCheckRange(0,4);
    }
}
