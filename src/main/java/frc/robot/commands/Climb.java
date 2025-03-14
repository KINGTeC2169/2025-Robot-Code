package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.wpilibj.Timer;

public class Climb extends Command{
    private Climber climber; 
    private double rpm;

    public Climb (Climber climber, double rpm) {
        this.climber = climber;
        this.rpm = rpm;

        addRequirements(climber);
    }

        // TODO: Unfinished
        // Called when the command is initially scheduled.
        @Override
        public void initialize() {
           
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
            return false; //TODO: Change this to the correct condition
        }
        
}
