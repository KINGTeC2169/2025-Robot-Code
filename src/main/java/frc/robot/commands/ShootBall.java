package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;

public class ShootBall extends Command {
    
    private Shooter shooter;
    private Intake index;
    private double rpm;

    private boolean shooterReady = false;
    
    public ShootBall(Shooter shooter, Intake index, double rpm) {
        this.shooter = shooter;
        this.index = index;
        this.rpm = rpm;
        addRequirements(shooter);
        addRequirements(index);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
       // shooter.vroom(20);
       shooter.setRPM(rpm);
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //shooter.vroom(20);
        //temporary if statement since we do not have a variable target velocity yet
        /* 
        if(shooter.getRPM() > 5000){
            shooterReady = true;
        }
        if(shooterReady){
            index.setVoltageIndex(9);
            index.setVoltageIntake(9);
        } */
       shooter.setRPM(rpm);
       
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Timer.delay(1);
        index.setVoltageIndex(9);
        index.setVoltageIntake(9);
        Timer.delay(0.75);
        shooter.setPower(0);
        index.setVoltageIndex(0);
        index.setVoltageIntake(0);
    }
    
    @Override
    public boolean isFinished() {
        return shooter.getRPM() > (rpm - 100);
    }

}

