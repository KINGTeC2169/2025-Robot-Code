// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.time.chrono.ThaiBuddhistChronology;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LED;
import frc.robot.util.Elastic;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final PowerDistribution pdh;
  private String autoName, newAutoName;
  private List<PathPlannerPath> pathPlannerPaths = null;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    pdh = new PowerDistribution(1, ModuleType.kRev);
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    SmartDashboard.putNumber("CAN Utilization %", RobotController.getCANStatus().percentBusUtilization * 100.0);
    SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
    SmartDashboard.putNumber("CPU Temperature", RobotController.getCPUTemp());
    SmartDashboard.putBoolean("RSL", RobotController.getRSLState());
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putData("pdh", pdh);

    //Swerve Widget
    SmartDashboard.putData("Swerve Drive", new Sendable() {
    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("SwerveDrive");

      builder.addDoubleProperty("Front Left Angle", () -> m_robotContainer.drivetrain.getModule(0).getCurrentState().angle.getRadians(), null);
      builder.addDoubleProperty("Front Left Velocity", () -> m_robotContainer.drivetrain.getModule(0).getCurrentState().speedMetersPerSecond, null);

      builder.addDoubleProperty("Front Right Angle", () -> m_robotContainer.drivetrain.getModule(1).getCurrentState().angle.getRadians(), null);
      builder.addDoubleProperty("Front Right Velocity", () -> m_robotContainer.drivetrain.getModule(1).getCurrentState().speedMetersPerSecond, null);

      builder.addDoubleProperty("Back Left Angle", () -> m_robotContainer.drivetrain.getModule(2).getCurrentState().angle.getRadians(), null);
      builder.addDoubleProperty("Back Left Velocity", () -> m_robotContainer.drivetrain.getModule(2).getCurrentState().speedMetersPerSecond, null);

      builder.addDoubleProperty("Back Right Angle", () -> m_robotContainer.drivetrain.getModule(3).getCurrentState().angle.getRadians(), null);
      builder.addDoubleProperty("Back Right Velocity", () -> m_robotContainer.drivetrain.getModule(3).getCurrentState().speedMetersPerSecond, null);

      builder.addDoubleProperty("Robot Angle", () -> m_robotContainer.drivetrain.getPigeon2().getRotation2d().getRadians(), null);
    }
  });
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
    @Override
    public void robotPeriodic() {
  //   // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
  //   // commands, running already-scheduled commands, removing finished or interrupted commands,
  //   // and running subsystem periodic() methods.  This must be called from the robot's periodic
  //   // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    if (DriverStation.isAutonomous()){
      Elastic.selectTab("Autonomous");
    }
    m_robotContainer.shooterSpeedTest = SmartDashboard.getNumber("Shooter Speed", 0);
   }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {

    newAutoName = m_robotContainer.getAutonomousCommand().getName();

    if (autoName != newAutoName) {
      autoName = newAutoName;
      if (AutoBuilder.getAllAutoNames().contains(autoName)) {
          
          try {
            pathPlannerPaths = PathPlannerAuto.getPathGroupFromAutoFile(newAutoName);
          } catch (IOException | ParseException e) {
            e.printStackTrace();
            pathPlannerPaths = null;
          }
          List<Pose2d> poses = new ArrayList<>();
          for (PathPlannerPath path : pathPlannerPaths) {
              if (DriverStation.getAlliance().get() == Alliance.Red) poses.addAll(path.flipPath().getAllPathPoints().stream().map(point -> new Pose2d(point.position.getX(), point.position.getY(), new Rotation2d())).collect(Collectors.toList()));
              else poses.addAll(path.getAllPathPoints().stream().map(point -> new Pose2d(point.position.getX(), point.position.getY(), new Rotation2d())).collect(Collectors.toList()));
            }
          m_robotContainer.logger.field.getObject("path").setPoses(poses);
      }
    }

    // if (DriverStation.isAutonomous()){
    //   if (pathPlannerPaths.get(0).getPathPoses().get(0).equals(Limelight.getLimelightPose())){
    //     //Set LEDs Green
    //   }
    //   else if (Limelight.getLimelightPose().getY() > pathPlannerPaths.get(0).getPathPoses().get(0).getY()){
    //     //Robot is too far to the left
    //     //Change right LEDs to different color
    //   }
    //   else if (Limelight.getLimelightPose().getY() < pathPlannerPaths.get(0).getPathPoses().get(0).getY()){
    //     //Robot is too far to the right
    //     //Change left LEDs to different color
    //   }
    // }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    Elastic.selectTab("Teleoperated");
    m_robotContainer.logger.field.getObject("path").setPoses();

    LED.intialize();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    if (m_robotContainer.topLeftButton.getAsBoolean()) m_robotContainer.setFastMode();
    else if (m_robotContainer.bottomRightButton.getAsBoolean()) m_robotContainer.setOverrideMode();
    else m_robotContainer.setMediumMode();
    // if (m_robotContainer.bottomRightButton.getAsBoolean()) m_robotContainer.setOverrideMode();
    // 
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
