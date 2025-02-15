// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

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

        //Pigeon CAN id
        public static final int pigeon = 4; //done

        //Swervedrive CAN ids
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
        
        public static final int encoder = 0;

    }

  public static final class ModuleConstants {
        //public static final double maxNeoSpeed = 3.68808;
        public static final double maxSpeed = 4.96824;
        public static final double maxNeoRadPerSec = 2 * 2 * Math.PI;
        public static final double wheelDiameter = 0.1016;//Units.inchesToMeters(4.0);
        public static final double driveGearRatio = 6.75; //1 / 6.12;
        public static final double turnGearRatio = 6.75; //1 / 12.8;
        public static final double driveMotorToMetersPerSec = driveGearRatio * Math.PI * wheelDiameter;
        public static final double turnEncoderToRadian = turnGearRatio * 2 * Math.PI;
        public static final double turnEncoderRPMToRadPerSec = turnEncoderToRadian;

        public static final double PDrive = 01;

        public static final double PTurn = 10;
        public static final double ITurn = 0;
        public static final double DTurn = 0;
    }

    public static final class DriveConstants {
        //These will need to be in meters
        public static final double rightLeftWheels = Units.inchesToMeters(23);
        public static final double frontBackWheels = Units.inchesToMeters(23);

        public static final double FRabsoluteOffset = -0.44332044769895; //2.670655153691769; //-0.470931150019169; //Post Season: 2.689068321163529
        public static final double FLabsoluteOffset = -1.972699293220935; //0.971007876098156; //-2.178248316049576; //Post Season: 1.135145783035374
        public static final double BRabsoluteOffset = 0.610524353578485; //-2.574014559388161; //0.572173677384853; //Post Season: -2.478912953223196
        public static final double BLabsoluteOffset = 0.612058334366371; //-2.503451585769653; //0.628930851817131; //Post Season: -2.486582857162624


    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(frontBackWheels / 2, rightLeftWheels / 2),//Front-Left
        new Translation2d(frontBackWheels / 2, -rightLeftWheels / 2),//Front-Right
        new Translation2d(-frontBackWheels / 2, rightLeftWheels / 2),//Back-Left
        new Translation2d(-frontBackWheels / 2, -rightLeftWheels / 2));//Back-Right
    }

    public static class IntakeConstants{

      public static final double encoderOffset = 0.857992571449814-0.5;
      public static final double rest = 0.857992571449814-0.5;
      public static final double net = 0.857992571449814-0.5;
      public static final double processor = 0.85; // fine tune for actual value      
    }
}



