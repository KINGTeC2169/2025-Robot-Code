// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Ports;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.ProcessorScoring;
import frc.robot.commands.Rev;
import frc.robot.commands.ShootBall;
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

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
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
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  //Commands
 public SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

  //Controller configurations
  // Replace with CommandPS4Controller or CommandJoystick if needed
  //private final CommandXboxController ps4 = new CommandXboxController(0);
  private final CommandXboxController m_driverController = new CommandXboxController(Ports.controller);
  private final Joystick leftStick = new Joystick(Constants.Ports.leftStick);
  private final JoystickButton topLeftButton = new JoystickButton(leftStick, 2);
  private final JoystickButton bottomLeftButton = new JoystickButton(leftStick, 1);

  //private final Joystick rightStick = new Joystick(Constants.Ports.rightStick);
  private final Joystick rightStick = new Joystick(Constants.Ports.rightStick);
  private final JoystickButton topRightButton = new JoystickButton(rightStick, 2);
  private final JoystickButton bottomRightButton = new JoystickButton(rightStick, 1);

  //More Swerve Constants
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.13).withRotationalDeadband(MaxAngularRate * 0.13) // Add a 13% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  public final Telemetry logger = new Telemetry(MaxSpeed);

  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

        NamedCommands.registerCommand("Rev", new Rev(shooter));
        NamedCommands.registerCommand("Shoot", new ShootBall(shooter, intake));
   // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
            
                /*drive.withVelocityX(-leftStick.getY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-leftStick.getX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-rightStick.getTwist() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            */
            drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed * 0.5) // Drive forward with negative Y (forward)
                    .withVelocityY(-m_driverController.getLeftX() * MaxSpeed * 0.5) // Drive left with negative X (left)
                    .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            
                    )
        );

        //Reset orientation
        m_driverController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //Defense mode
        bottomLeftButton.whileTrue(drivetrain.applyRequest(() -> brake));

        drivetrain.registerTelemetry(logger::telemeterize);

      SmartDashboard.putData("Auto Mode", autoChooser);        

      configureBindings();
    
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

    
    m_driverController.rightBumper().whileTrue((new ShootBall(shooter, intake)));
    m_driverController.leftBumper().whileTrue(new Rev(shooter));
    m_driverController.a().onTrue(new IntakeBall(intake));
    m_driverController.b().whileTrue(new ProcessorScoring(intake));
    m_driverController.y().whileTrue(Commands.run(() -> intake.outTake()));
    m_driverController.x().whileFalse(Commands.run(() -> intake.stopTake()));

    m_driverController.rightTrigger(.05).whileTrue((Commands.run(() -> intake.setVoltagePivot(m_driverController.getRightTriggerAxis()*4))));
    m_driverController.leftTrigger(.05).whileTrue((Commands.run(() -> intake.setVoltagePivot(-12))));
    m_driverController.start().whileTrue(Commands.run(() -> intake.setVoltagePivot(0)));
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
