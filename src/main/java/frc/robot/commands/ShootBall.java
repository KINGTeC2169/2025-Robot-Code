package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;

// just remove all the useless stuff
public class ShootBall extends Command {
    
    private Shooter shooter;
    private Intake intake;
    private double rpm;
    private int counter;
    private Timer timer;
    private double timerStartTime;

    private boolean shooterReady = false;
    
    // Creates a new ShootBall command to run the shooter and intake to shoot the ball
    public ShootBall(Shooter shooter, Intake intake, double rpm) {
        this.shooter = shooter;
        this.intake = intake;
    
        this.rpm = rpm;
        addRequirements(shooter);
        addRequirements(intake);

    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
       shooter.setTargetRPM(rpm);
       intake.setIntakePos(IntakeConstants.restball); // set the intake to restball position
       timer = new Timer();
       timer.start();
       counter = 0;
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(shooter.isReady()){ //if the shooter is ready to shoot, set the intake to run at a certain voltage to shoot the ball
            intake.setVoltageIntake(0.7*12);
            intake.setVoltageIndex(-0.7*12);
            if (timerStartTime == 0) timerStartTime = timer.get();
        } 
        shooter.setTargetRPM(rpm); // set the target RPM to the desired RPM to shoot the ball
       
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.setVoltageIntake(0); // set the intake and shooter to stop shooting
        intake.setVoltageIndex(0);
        shooter.setTargetRPM(0);
        intake.setIntakePos(IntakeConstants.rest); // set the intake to rest position
    }
    
    @Override
    public boolean isFinished() {
        return timer.get() > timerStartTime + 1; // check if the timer is greater than 1 second to end the command, this will give a short burst to shoot the ball
    }

}

