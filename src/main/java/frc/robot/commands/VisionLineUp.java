package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;


public class VisionLineUp extends Command{
    private Intake intake;
    private Shooter shooter;
    private Timer timer;
    private double difference;
    
    
    
    public VisionLineUp(Intake intake, Shooter shooter){
        this.intake = intake;
        addRequirements(intake);
        this.shooter = shooter;
        addRequirements(shooter);
        timer = new Timer();
        Constants.Vision.dif1 = Limelight.distanceFromTag() - 138;
    }

    @Override
    public void initialize(){
        intake.shouldOuttakeAdjust = true;
        Constants.Vision.dif1 = Limelight.distanceFromTag() - 138;
    }

    @Override
    public void execute(){
        Constants.Vision.dif1 = Limelight.distanceFromTag() - 138;
    }

    @Override
    public void end(boolean interrupted){
        intake.shouldOuttakeAdjust = false;
        
        
    }

    @Override
    public boolean isFinished(){
      
        return false;//intake.getDistance();
    }
}
