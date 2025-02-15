package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.Swerve;

public class DriveCommand extends Command{
    
    private final Swerve swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> resetEncoder, resetPigeon, isSlow, isFast;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    
    public DriveCommand(Swerve swerveSubsystem, Supplier<Double> xSpdFunction, 
        Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, Supplier<Boolean> resetEncoder, 
        Supplier<Boolean> resetPigeon, Supplier<Boolean> isSlow, Supplier<Boolean> isFast){
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.resetEncoder = resetEncoder;
        this.resetPigeon = resetPigeon;
        this.isSlow = isSlow;
        this.isFast = isFast;
        this.xLimiter = new SlewRateLimiter(5);
        this.yLimiter = new SlewRateLimiter(5);
        this.turningLimiter = new SlewRateLimiter(5);
        addRequirements(swerveSubsystem);
    }

	@Override
    public void initialize(){
    }

    @Override
    public void execute() {
        //Gets the speeds from controller input
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        ChassisSpeeds chassisSpeeds;

        // Deadband: unsure if necessary for our controllers
        xSpeed = Math.abs(xSpeed) > .07 ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > .07 ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > .08 ? turningSpeed : 0.0;

        xSpeed = xLimiter.calculate(xSpeed) *  Constants.ModuleConstants.maxSpeed;
        ySpeed = yLimiter.calculate(ySpeed) *  Constants.ModuleConstants.maxSpeed;
        turningSpeed = turningLimiter.calculate(turningSpeed) * ModuleConstants.maxNeoRadPerSec;

        //Slow mode
        if (isSlow.get()){
            xSpeed *= swerveSubsystem.getSlowSpeed();
            ySpeed *= swerveSubsystem.getSlowSpeed();
            turningSpeed *= swerveSubsystem.getSlowSpeed();
            //swerveSubsystem.oneRotation(); For testing yesterday. 
        }

        //Fast mode
        else if (isFast.get()){
            xSpeed *= swerveSubsystem.getFastSpeed();
            ySpeed *= swerveSubsystem.getFastSpeed();
            turningSpeed *= swerveSubsystem.getFastSpeed(); 
        }

        //Regular speed if no other controller buttons are pressed
        else {
            xSpeed *= swerveSubsystem.getMediumSpeed();
            ySpeed *= swerveSubsystem.getMediumSpeed();
            turningSpeed *= swerveSubsystem.getMediumSpeed();
        }

        //Resets swerve drive encoders
        if (resetEncoder.get()) {
            swerveSubsystem.resetEncoders();
        }

        //Resets pigeon
        if (resetPigeon.get()) {
            swerveSubsystem.zeroHeading();
        }

        //Sets robot to field relative mode
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, Pigeon.getRotation2d());

        //Sets robot to robot relative mode 
        //DO NOT UNCOMMENT UNLESS YOU KNOW WHAT YOU ARE DOING
        //chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(xSpeed, ySpeed, turningSpeed, Pigeon.getRotation2d());

        SwerveModuleState[] moduleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds); 
        
        swerveSubsystem.setModuleStates(moduleStates);

    }
    
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

  
    @Override
    public boolean isFinished() {
        return false;
    }
}