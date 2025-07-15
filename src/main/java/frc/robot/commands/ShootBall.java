package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj.Timer;

// just remove all the useless stuff
public class ShootBall extends Command {
    
    private Shooter shooter;
    private Intake intake;
    private LED led;
    private double rpm;
    private int counter;
    private Timer timer;

    private boolean shooterReady = false;
    
    // Creates a new ShootBall command to run the shooter and intake to shoot the ball
    public ShootBall(Shooter shooter, Intake intake, LED led, double rpm) {
        this.shooter = shooter;
        this.intake = intake;
        this.led = led;
        this.rpm = rpm;
        timer = new Timer();
        addRequirements(shooter, intake, led);

    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
       led.setBlinkingBlue();
       // shooter.vroom(20);
       shooter.setTargetRPM(rpm);
       intake.setIntakePos(IntakeConstants.restball); // set the intake to restball position
       timer = new Timer();
       counter = 0;
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {        
        if(shooter.isReady()) counter++; //counter creates a delay to make sure the shooter is ready to shoot and 
        if(counter == 10)shooterReady = true; 
        if(shooterReady){ //if the shooter is ready to shoot, set the intake to run at a certain voltage to shoot the ball
            Constants.DriveConstants.breakMode = true; // set the drive to break mode to stop the robot from moving while shooting the ball
            led.setGreen(); //turns on the LED to show that the shooter is shooting
            intake.setVoltageIntake(0.7*12);
            intake.setVoltageIndex(-0.7*12);
            shooterReady = false;
            if (!shooterReady){
                timer.start();
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
        led.setRed(); //turns off the LED to show that the robot no longer contains a ball
    }

    @Override
    public boolean isFinished() {
        return timer.get() > 1; // check if the timer is greater than 1 second to end the command, this will give a short burst to shoot the ball
    }

}