// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Ports;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.ProcessorScoring;
import frc.robot.commands.Rev;
import frc.robot.commands.ShootBall;
import frc.robot.commands.JustIntakeBallNoSensor;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.*;

import java.util.jar.Attributes.Name;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.util.Named;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  //Subsystems
  public final Shooter shooter = new Shooter();
  public final Intake intake = new Intake();
  public final CommandSwerveDrivetrain drivetrain;

  //Commands
 public SendableChooser<Command> autoChooser;
  private double speed = 0.5;
  public double shooterSpeedTest = 0;

  //Controller configurations
  // Replace with CommandPS4Controller or CommandJoystick if needed
  //private final CommandXboxController ps4 = new CommandXboxController(0);
  private final CommandXboxController m_driverController = new CommandXboxController(Ports.controller);
  private final Joystick leftStick = new Joystick(Constants.Ports.leftStick);
  public final JoystickButton topLeftButton = new JoystickButton(leftStick, 1);
  public final JoystickButton bottomLeftButton = new JoystickButton(leftStick, 2);

  //private final Joystick rightStick = new Joystick(Constants.Ports.rightStick);
  private final Joystick rightStick = new Joystick(Constants.Ports.rightStick);
  private final JoystickButton topRightButton = new JoystickButton(rightStick, 1);
  private final JoystickButton bottomRightButton = new JoystickButton(rightStick, 2);

  //More Swerve Constants
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.10).withRotationalDeadband(MaxAngularRate * 0.10) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  public final Telemetry logger = new Telemetry(MaxSpeed);

  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

        NamedCommands.registerCommand("Rev", new Rev(shooter, 5000));
        NamedCommands.registerCommand("Shoot", new ShootBall(shooter, intake, 5000));
        NamedCommands.registerCommand("Intake", new IntakeBall(intake,shooter));
        NamedCommands.registerCommand("ProcessorScoring", new ProcessorScoring(intake));

        drivetrain = TunerConstants.createDrivetrain();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);    
        SmartDashboard.putNumber("Swerve Speed", speed);
        SmartDashboard.putNumber("Shooter Speed", shooterSpeedTest);
        
   // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
            drive.withVelocityX(-leftStick.getY() * MaxSpeed * speed) // Drive forward with negative Y (forward)
                    .withVelocityY(-leftStick.getX() * MaxSpeed * speed) // Drive left with negative X (left)
                    .withRotationalRate(rightStick.getTwist() * MaxAngularRate * speed * 2) // Drive counterclockwise with negative X (left)
                    )

            // drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed * 0.5) // Drive forward with negative Y (forward)
            //         .withVelocityY(-m_driverController.getLeftX() * MaxSpeed * 0.5) // Drive left with negative X (left)
            //         .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate * 0.5) // Drive counterclockwise with negative X (left)        
            //         )
        );

        //Reset orientation
        topRightButton.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //Defense mode
        bottomRightButton.whileTrue(drivetrain.applyRequest(() -> brake));

        drivetrain.registerTelemetry(logger::telemeterize);    

      configureBindings();
    
  }

  public void setFastMode(){
    speed = 1;
  }

  public void setMediumMode(){
    speed = SmartDashboard.getNumber("Swerve Speed", 0.5);
  }

  public void setSlowMode(){
    speed = 0.2;
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    
    m_driverController.rightBumper().onTrue((new ShootBall(shooter, intake, 5500)));
    m_driverController.leftBumper().onTrue(new Rev(shooter, 5500));
    m_driverController.a().onTrue(new IntakeBall(intake,shooter));
    m_driverController.b().onTrue(new ProcessorScoring(intake));
    m_driverController.y().whileTrue(Commands.run(() ->intake.shouldOuttake = true));
    m_driverController.x().whileTrue(Commands.run(() -> intake.shouldOuttake
     = false));
    // m_driverController.y().whileTrue(Commands.run(() -> intake.setVoltageIndex(0.3)));
    // m_driverController.x().whileFalse(Commands.run(() -> intake.setVoltageIndex(-0.3)));

    m_driverController.pov(0).whileTrue(Commands.run(() -> intake.setIntakePos(Constants.IntakeConstants.rest)));
    m_driverController.pov(180).whileTrue(Commands.run(() -> intake.setIntakePos(Constants.IntakeConstants.grab)));
    m_driverController.start().whileTrue(Commands.run(() -> intake.setVoltagePivot(0)));
    // m_driverController.b().whileTrue(Commands.run(() -> shooter.setRPM(-1500)));
    //m_driverController.x().whileTrue(Commands.run(() -> intake.setVoltagePivot(m_driverController.getRightTriggerAxis())));
    //m_driverController.y().whileTrue(Commands.run(() -> intake.setVoltagePivot(-m_driverController.getLeftTriggerAxis())));
    m_driverController.start().whileTrue(Commands.run(() -> intake.setVoltagePivot(0)));
    // m_driverController.a().whileTrue(Commands.run(() -> shooter.setRPM(0)));

    // m_driverController.rightTrigger(.05).onTrue(Commands.runOnce(SignalLogger::start));
    // m_driverController.leftTrigger(.05).onTrue(Commands.runOnce(SignalLogger::stop));

    /*
    * Joystick Y = quasistatic forward
    * Joystick A = quasistatic reverse
    * Joystick B = dynamic forward
    * Joystick X = dyanmic reverse
    */
    // m_driverController.y().whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // m_driverController.a().whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // m_driverController.b().whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // m_driverController.x().whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));

  }
  public void runIntake(){
    intake.setMotorDistanceSensor();
  }
  public void runShooter(){
    shooter.setMotorWithPID();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //An example command will be run in autonomous
    return autoChooser.getSelected();
 }
}
