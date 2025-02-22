package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;

public class ShootBall extends Command {
    
    private Shooter shooter;
    private Intake index;
    private boolean shooterReady = false;
    
    public ShootBall(Shooter shooter, Intake index) {
        this.shooter = shooter;
        this.index = index;
        addRequirements(shooter);
        addRequirements(index);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shooter.vroom(20);
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.vroom(20);
        //temporary if statement since we do not have a variable target velocity yet
        if(shooter.getVelocityFly() > 19.5){
            shooterReady = true;
        }
        if(shooterReady){
            index.setVoltageIndex(9);
        }
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        index.setVoltageIndex(9);
        Timer.delay(0.75);
        shooter.setPower(0);
        index.setVoltageIndex(0);
    }
    
    @Override
    public boolean isFinished() {
        return shooter.getRPM() > 4800;
    }

}

