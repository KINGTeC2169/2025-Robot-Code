package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.Timer;

// just remove all the useless stuff
public class ShootBall extends Command {
    
    private Shooter shooter;
    private Intake intake;
    private double rpm;
    private int counter;
    private Timer timer;
    private double t0;

    private boolean shooterReady = false;
    
    public ShootBall(Shooter shooter, Intake intake, double rpm) {
        this.shooter = shooter;
        this.intake = intake;
    
        this.rpm = rpm;
        timer = new Timer();
        addRequirements(shooter);
        addRequirements(intake);

    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
       // shooter.vroom(20);
       shooter.setTargetRPM(rpm);
       intake.setIntakePos(IntakeConstants.restball);
       timer = new Timer();
       counter = 0;




       
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //shooter.vroom(20);
        //temporary if statement since we do not have a variable target velocity yet
         
        if(shooter.isReady())counter++;
        if(counter == 10)shooterReady = true;
        if(shooterReady){
            intake.setVoltageIntake(0.7*12);
            intake.setVoltageIndex(-0.7*12);
            shooterReady = false;
            if (!shooterReady){
                timer.start();
            }
        } 
        shooter.setTargetRPM(rpm);
        
        //System.out.println(timer.get());
       
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.setVoltageIntake(0);
        intake.setVoltageIndex(0);
        shooter.setTargetRPM(0);
        intake.setIntakePos(IntakeConstants.rest);
    }
    
    @Override
    public boolean isFinished() {
        return timer.get() > 1;
    }

}

