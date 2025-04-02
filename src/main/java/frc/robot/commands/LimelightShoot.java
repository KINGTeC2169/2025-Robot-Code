package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Limelight;

// just remove all the useless stuff
public class LimelightShoot extends Command {
    
    private Shooter shooter;
    private Intake intake;
    private double rpm;
    private int counter;
    private Timer timer;
    private int step;

    private boolean shooterReady = false;
    
    // Creates a new ShootBall command to run the shooter and intake to shoot the ball
    public LimelightShoot(Shooter shooter, Intake intake, double rpm) {
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
       LED.setBlue();
       
       
       intake.setIntakePos(IntakeConstants.restball); // set the intake to restball position
       timer = new Timer();
       shooter.setTargetRPM(rpm);
       counter = 0;
        step = 0;



       
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {        

        // if(!Limelight.getTv()) step == 0;
        // if(Limelight.getTx() > Math.abs()
        // else dif = 0;
        if(step == 3){
            if(shooter.isReady())counter++; //counter creates a delay to make sure the shooter is ready to shoot and 
            if(counter == 10)shooterReady = true; 
            if(shooterReady){ //if the shooter is ready to shoot, set the intake to run at a certain voltage to shoot the ball
                Constants.DriveConstants.breakMode = true; // set the drive to break mode to stop the robot from moving while shooting the ball
                LED.setGreen(); //turns on the LED to show that the shooter is shooting
                intake.setVoltageIntake(0.7*12);
                intake.setVoltageIndex(-0.7*12);
                shooterReady = false;
                if (!shooterReady){
                    timer.start();
                }
            } 
        }
        shooter.setTargetRPM(rpm); // set the target RPM to the desired RPM to shoot the ball
       
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Constants.DriveConstants.breakMode = false; // set the drive to coast mode to allow the robot to move after shooting the ball
        intake.setVoltageIntake(0); // set the intake and shooter to stop shooting
        intake.setVoltageIndex(0);
        shooter.setTargetRPM(0);
        intake.setIntakePos(IntakeConstants.rest); // set the intake to rest position
        LED.setRed(); //turns off the LED to show that the shooter is not running
    }
    
    @Override
    public boolean isFinished() {
        return !Limelight.getTv() || timer.get() > 1; // check if the timer is greater than 1 second to end the command, this will give a short burst to shoot the ball
    }

}