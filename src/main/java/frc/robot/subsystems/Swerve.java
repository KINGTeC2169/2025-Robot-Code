package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.Ports;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;


public class Swerve extends SubsystemBase {

    //Creates an instance of SwerveModule for each module on the robot
    private SwerveModule frontLeft = new SwerveModule(
    Ports.frontLeftDrive,//Drive wheel id
    Ports.frontLeftTurn, //Turn wheel id
    true, false,
    Ports.frontLeftAbsolute,
    DriveConstants.FLabsoluteOffset,
    false);

    private SwerveModule frontRight = new SwerveModule(
    Ports.frontRightDrive,
    Ports.frontRightTurn, 
    true, false,
    Ports.frontRightAbsolute,
    DriveConstants.FRabsoluteOffset,
    false);

    private SwerveModule backLeft = new SwerveModule(
    Ports.backLeftDrive,
    Ports.backLeftTurn, 
    true, false,
    Ports.backLeftAbsolute,
    DriveConstants.BLabsoluteOffset,
    false);

    private SwerveModule backRight = new SwerveModule(
    Ports.backRightDrive,
    Ports.backRightTurn, 
    true, false,
    Ports.backRightAbsolute,
    DriveConstants.BRabsoluteOffset,
    false);

    public SwerveDriveKinematics kinematics = DriveConstants.DRIVE_KINEMATICS;
    private final SwerveDriveOdometry odometer;

    private double fastSpeed = 0.8;
    private double mediumSpeed = 0.5;
    private double slowSpeed = 0.2;
    
    public static Field2d field = new Field2d();

   
    public Swerve() {
        odometer = new SwerveDriveOdometry(kinematics, getRotation2d(), getModulePositions(), new Pose2d(0, 0, new Rotation2d(0)));
        
        //Shuffleboard data        
        SmartDashboard.putNumber("Front Left", frontLeft.getDriveCurrent());
        SmartDashboard.putNumber("Front Right", frontRight.getDriveCurrent()); 
        SmartDashboard.putNumber("Back Left", backLeft.getDriveCurrent());
        SmartDashboard.putNumber("Back Right", backRight.getDriveCurrent()); 

        SmartDashboard.putNumber("Front Left", frontLeft.getTurnCurrent());
        SmartDashboard.putNumber("Front Right", frontRight.getTurnCurrent());
        SmartDashboard.putNumber("Back Left", backLeft.getTurnCurrent());
        SmartDashboard.putNumber("Back Right", backRight.getTurnCurrent());

        SmartDashboard.putNumber("Front Left Absolute", frontLeft.getAbsoluteTurnPosition());
        SmartDashboard.putNumber("Front Right Absolute", frontRight.getAbsoluteTurnPosition());
        SmartDashboard.putNumber("Back Left Absolute", backLeft.getAbsoluteTurnPosition());
        SmartDashboard.putNumber("Back Right Absolute", backRight.getAbsoluteTurnPosition());

        SmartDashboard.putNumber("Fast Speed", getFastSpeed());
        SmartDashboard.putNumber("Medium Speed",getMediumSpeed());
        SmartDashboard.putNumber("Slow Speed", getSlowSpeed());
        

        SmartDashboard.putNumber("Robot Heading", getHeading());

        SmartDashboard.putData(field);

        // Configure AutoBuilder last
    //     AutoBuilder.configureHolonomic(
    //         this::getPose, // Robot pose supplier
    //         this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
    //         this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //         this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
    //         new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
    //                 new PIDConstants(1.95, 0.0, 0.0), // Translation PID constants
    //                 new PIDConstants(2.35, 0.0, 0.0), // Rotation PID constants
    //                 ModuleConstants.maxSpeed, // Max module speed, in m/s
    //                 0.291, // Drive base radius in meters. Distance from robot center to furthest module.
    //                 new ReplanningConfig() // Default path replanning config. See the API for the options here
    //         ),
    //         () -> {
    //           // Boolean supplier that controls when the path will be mirrored for the red alliance
    //           // This will flip the path being followed to the red side of the field.
    //           // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //           var alliance = DriverStation.getAlliance();
    //           if (alliance.isPresent()) {
    //             return alliance.get() == DriverStation.Alliance.Red;
    //           }
    //           return true;
    //         },
    //         this // Reference to this subsystem to set requirements
    // );

        //Creates a new thread, which sleeps and then zeros out the gyro
        //Uses a new thread so that it doesn't pause all other code running
        new Thread(() -> {
            try {
                Thread.sleep(2000);
                zeroHeading();
                resetEncoders();
            } catch (Exception e) {
            }
        }).start();
    }

    /**Returns the field. */
    public Field2d getField() {
        return field;
    }

    /**Returns the fast speed, which is adjustable via the slider on shuffleboard.*/
    public double getFastSpeed() {
        return fastSpeed;
    }

    /**Returns the medium speed, which is adjustable via the slider on shuffleboard.*/
    public double getMediumSpeed() {
        return mediumSpeed;
    }

    /**Returns the slow speed, which is adjustable via the slider on shuffleboard.*/
    public double getSlowSpeed() {
        return slowSpeed;
    }
    
    /**Returns the Module positions of the 4 swerve modules in the order frontLeft, frontRight, backLeft, backRight.*/
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
        frontLeft.getModulePosition(),
        frontRight.getModulePosition(),
        backLeft.getModulePosition(),
        backRight.getModulePosition()
        };
        
    }

    /**Resets the Pigeon.*/
    public void zeroHeading() {
        //System.out.println("Zeroing gyro \n.\n.\n.\n.\n.\n.\n.");
        Pigeon.reset();
    }

    /**Returns the heading of the pigeon.*/
    public double getHeading() {
        //return Math.IEEEremainder(NavX.getAngle(), 360);
        return Pigeon.getYaw() % 360;
    }

    /**Returns the rotation2d from the pigeon.*/
    public Rotation2d getRotation2d() {
        return Pigeon.getRotation2d();
        //return Rotation2d.fromDegrees(-getHeading());
    }

    /**Returns chassis speeds that are relative to the robot's front.*/
    public ChassisSpeeds getRobotRelativeSpeeds(){
        return kinematics.toChassisSpeeds(getModuleStates()[0], 
                                          getModuleStates()[1], 
                                          getModuleStates()[2], 
                                          getModuleStates()[3]);
    }

    /**Sets swerve module states relative to the robot's front. */
    public void driveRobotRelative(ChassisSpeeds speeds){
        setModuleStates(kinematics.toSwerveModuleStates(speeds));
    }

    /**Returns the odometer's position in the form of a Pose2d. */
    public Pose2d getPose() {
        //return NavX.getPose();
        return odometer.getPoseMeters();
    }

    /**Resets the position of the odometer using the rotation2d of the pigeon, the module positions, and a pose2d. */
    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    /**Resets the encoders of the 4 swerve modules. */
    public void resetEncoders() {
        System.out.println("reset");
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backRight.resetEncoders();
        backLeft.resetEncoders();
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), getModulePositions());

        //field.setRobotPose(odometer.getPoseMeters());
        field.setRobotPose(odometer.getPoseMeters().getX(), odometer.getPoseMeters().getY(), odometer.getPoseMeters().getRotation());
        SmartDashboard.putData("Swerve Drive", new Sendable() {
        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("SwerveDrive");

            builder.addDoubleProperty("Front Left Angle", () -> frontLeft.getAngle().getRadians(), null);
            builder.addDoubleProperty("Front Left Velocity", () -> frontLeft.getDriveVelocity(), null);

            builder.addDoubleProperty("Front Right Angle", () -> frontRight.getAngle().getRadians(), null);
            builder.addDoubleProperty("Front Right Velocity", () -> frontRight.getDriveVelocity(), null);

            builder.addDoubleProperty("Back Left Angle", () -> backLeft.getAngle().getRadians(), null);
            builder.addDoubleProperty("Back Left Velocity", () -> backLeft.getDriveVelocity(), null);

            builder.addDoubleProperty("Back Right Angle", () -> backRight.getRotation2d().getRadians(), null);
            builder.addDoubleProperty("Back Right Velocity", () -> backRight.getDriveVelocity(), null);

            builder.addDoubleProperty("Robot Angle", () -> getRotation2d().getRadians(), null);
        }
        });
    }

    /**Stops all 4 swerve modules. */
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

     /**Takes an array of SwerveModuleStates and sets each SwerveModule to its respective state */
     public void setModuleStates(SwerveModuleState[] states) {

        SwerveDriveKinematics.desaturateWheelSpeeds(states, ModuleConstants.maxSpeed);
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);

    }

    /**Sets modules states for autos only. */
    public void setAutoModuleStates(SwerveModuleState[] states) {

        SwerveDriveKinematics.desaturateWheelSpeeds(states, ModuleConstants.maxSpeed);
        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        backLeft.setState(states[2]);
        backRight.setState(states[3]);

    }

    /**Gets all 4 swerve module states. */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()};
    }

    /**Stops the wheels. Untested. Not used.*/
    public void fullStop() {
        frontLeft.fullStop();
        frontRight.fullStop();
        backLeft.fullStop();
        backRight.fullStop();
    }

    /**Puts wheels in 'X' position and sets driving to a velocity-PID loop set at 0m/s */
    public void setActiveStop() {
        //System.out.println("1\n1\n1\n1\n1\n1\n1\n1");
        frontLeft.activeStop(-1);
        frontRight.activeStop(1);
        backLeft.activeStop(1);
        backRight.activeStop(-1);
    }

    public void playNote(double hz){
        frontLeft.playNote(hz);
        frontRight.playNote(hz);
        backLeft.playNote(hz);
        backRight.playNote(hz);
    }
}