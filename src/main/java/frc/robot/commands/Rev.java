package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;

public class Rev extends Command {
    
    private Shooter shooter;
    private Intake intake;
    private double rpm;
    
    public Rev(Shooter shoot, double rpm, Intake intake){
        shooter = shoot;
        addRequirements(shooter);
        this.intake = intake;
        addRequirements(intake);
        this.rpm = rpm;
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        intake.setIntakePos(IntakeConstants.restball); // set the intake to restball position
        LED.shootingREVing(); //turns on the LED to show that the shooter is running
        shooter.setTargetRPM(rpm); // set the target RPM to the desired RPM to run the shooter at
        
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }
    
    
    @Override
    public boolean isFinished() {
        return shooter.isReady();
    }
    

}

