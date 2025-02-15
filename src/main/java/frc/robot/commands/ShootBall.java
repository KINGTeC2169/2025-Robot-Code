package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

public class ShootBall extends Command {
    
    private Shooter shooter;
    private Intake index;
    private double start;
    private boolean done;
    
    public ShootBall(Shooter shooter, Intake index) {
        this.shooter = shooter;
        this.index = index;
        addRequirements(shooter);
        addRequirements(index);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        index.setVoltageIndex(9);
        start = System.currentTimeMillis();
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //after 2 seconds end the index spin
        if(System.currentTimeMillis() - start > 2000) {
            index.setVoltageIndex(0);
        }
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.setPower(0);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }

}

