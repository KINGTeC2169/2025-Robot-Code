package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class SelfTest extends Command {
    private CommandSwerveDrivetrain swerveSubsystem;
    private Shooter shooter;
    private Intake intake;

    private SwerveRequest.RobotCentric driveForward = new SwerveRequest.RobotCentric().withVelocityX(1);
    private SwerveRequest.RobotCentric driveSideways = new SwerveRequest.RobotCentric().withVelocityY(1);
    private SwerveRequest.RobotCentric stop = new SwerveRequest.RobotCentric();

    
    public SelfTest(CommandSwerveDrivetrain swerveSubsystem, Shooter shooter, Intake intake){
        this.swerveSubsystem = swerveSubsystem;
        this.shooter = shooter;
        this.intake = intake;
        addRequirements(swerveSubsystem, shooter, intake);
    }

    @Override
    public void execute(){
        System.out.println("--------------------------------------------------------------------------------");
        System.out.println("-----------------------Starting Morgan Le Fay's Self Test-----------------------");
        System.out.println("--------------------------------------------------------------------------------\n\n\n");

        System.out.println("Testing Swerve Subsystem...");
        swerveSubsystem.applyRequest(() -> driveForward);
        Timer.delay(2);
        swerveSubsystem.applyRequest(() -> stop);
        swerveSubsystem.applyRequest(() -> driveSideways);
        swerveSubsystem.applyRequest(() -> stop);
        
        System.out.println("Testing Intake Subsystem...");
        intake.setIntakePos(IntakeConstants.grab);
        Timer.delay(0.5);
        intake.setIntakePos(IntakeConstants.rest);
        Timer.delay(0.5);
        intake.setIntakePos(IntakeConstants.restball);
        Timer.delay(0.5);
        intake.setVoltageIntake(0.3);
        Timer.delay(0.5);
        intake.setVoltageIntake(0);
        intake.setVoltageIndex(0.5);
        Timer.delay(0.5);
        intake.setVoltageIndex(0);
        System.out.println("ALL INTAKE COMMANDS EXECUTED\n");

        System.out.println("Testing Shooter Subsystem...");
        shooter.setTargetRPM(3000);
        Timer.delay(2);
        shooter.setTargetRPM(0);
        System.out.println("SHOOTER SUBSYSTEM TEST COMPLETE\n");
    }

    @Override
    public void end(boolean interrupted){
        System.out.println("-------------------------------------------------------------------------");
        System.out.println("----------------------------Self Test Complete---------------------------");
        System.out.println("-------------------------------------------------------------------------\n\n\n"); 
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}