// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

  public static final class Ports {
        
        //Driver station controller ids
        public static final int leftStick = 0;
        public static final int rightStick = 1;
        public static final int controller = 2;
        public static final int buttons = 3;

        //Pigeon CAN id (NOT USED)
        public static final int pigeon = 4; //done

        //Swervedrive CAN ids (NOT USED)
        public static final int frontLeftDrive = 9; //done
        public static final int frontLeftTurn = 8; //done
        public static final int frontLeftAbsolute = 19; //done
        public static final int frontRightDrive = 13; //done
        public static final int frontRightTurn = 12; //done
        public static final int frontRightAbsolute = 18; //done
        public static final int backLeftDrive = 10; //done
        public static final int backLeftTurn = 11; // done
        public static final int backLeftAbsolute = 17; //done
        public static final int backRightDrive = 16; //done
        public static final int backRightTurn = 15; //done
        public static final int backRightAbsolute = 20; //done

        //Shooter CAN ids
        

        //Intake CAN id 
        public static final int shooterMotor = 14; //done
        public static final int intakeMotor = 7; //done
        public static final int pivotMotor = 5; //done
        public static final int indexerMotor = 6; //done
        public static final int reefIntakeMotor = 22; 
        public static final int reefPivotMotor = 21; 
        public static final int climberMotor = 23; 
        
        public static final int intakeHexPort = 1;
        public static final int reefHexPort = 2;

        public static final Port arduino = SerialPort.Port.kUSB1;
        

    }

    public static final class DriveConstants {
      public static RobotConfig config;
      public static PIDConstants autoTranslationPID = new PIDConstants(5.0, 0, 0);
      public static PIDConstants autoRotationPID = new PIDConstants(5.0, 0, 0);

      public static boolean breakMode = false;
    }

    public static class IntakeConstants{

      public static final double encoderOffset = 0;
      public static final double rest = 0.1776698044417451;
      public static final double grab = 0.06343960158599005;
      public static final double restball = 0.14493320312333007;
      //TODO: Tune values
      public static double kP = 20.0;
      public static double kI = 0;
      public static double kD = 0;
      
      public static double kS = 0;
      public static double kV = 3.5;
      public static double kA = 0;
 
      public static double kPdown = 20.0;
      public static double kIdown = 0;
      public static double kDdown = 0;

      public double shootBallPivotPos = 0;
    }

    public static class ReefIntakeConstants{
// tune all of these this is just temporary                           
      public static final double encoderOffset = 0.3063192076579802 - 0.25;
      public static final double reefrest = 0.1776698044417451;
      public static final double reefgrab = 0.06343960158599005;

      public static double kP = 0;
      public static double kI = 0;
      public static double kD = 0;
      
      public static double kS = 0.01;
      public static double kV = 0.02;
      public static double kG = 4.05; //adjust this if arm to high increase 
 
      public static double kPdown = 20.0;
      public static double kIdown = 0;
      public static double kDdown = 0;
    }  
    
    public static class ClimberConstants{
      public static final double encoderOffset = 0;
      public static final double rest = 0.1776698044417451;
      public static final double takeOff = 0.06343960158599005;

      public static double kP = 20.0;
      public static double kI = 0;
      public static double kD = 0;

      public static double kS = 0;
      public static double kV = 3.5;
      public static double kA = 0;

      public static double kPdown =20.0;
      public static double kIdown = 0;
      public static double kDdown = 0;
    }

    public static class ShooterConstants{

      public static double kP = 0.09; //1.01;
      public static double kI = 0;
      public static double kD = 0;
      
      public static double kS = 0.1;
      public static double kV = 0.12325;
    }

    public static class Vision{
      public static double LLAngle = 30;
      public static double bargeTagHeight = 70;
      public static double LLHeight = 28;

      public static double dif1 = 0;
    }
}



