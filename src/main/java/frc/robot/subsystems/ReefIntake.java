package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.ReefIntakeConstants;

/* How to tune the reef intake:
 * YOU SHOULD NOT HAVE TO ADD OR DELETE ANY LINES OF CODE, just uncomment and change values when you get to them.
 * First setup the hex encoder
 * 1. Manually hold the pivot straight up so it balances on itself and take that postion.
 * 2. Then Hold the pivot straight out in front of the robot and take that position.
 * 3. If the 2nd position is less then the first, then the encoder is backwards, change getPosition() to -encoder.get() 
 * then repeat steps 1 and 2
 * 4. Now your encoder offset is: 1st position - 0.25. If this value is less then 0 add 1.
 * 5. Verify that the encoder offset is correct by manually moving the arm straight up and seeing it reads 0.25. 
 * HEX ENCODER IS TUNED!!!!!!!
 * 
 * Tune PID 
 * 1. You can manually move the reef intake with either trigger.
 * 2. In elastic use the reef pivot PID to tune the PID.
 * 3. You can change the set position of the reef intake with x and y.
 * If the Arm is moving the opposite direction of the set point put a negative in front of the pivotPID line 135
 * 
 * Once you are done PID tuning you can test the flywheel, (button board top left 3 buttons)
 * Then test the reef knock off command. Comment out the manual set position in robot container and uncomment the Reef knock off command.
 * When you test the reef knock off command you may need to chang ethe flywheel volts which can be changed in ReefKnockOff.java Line 25k
 */

public class ReefIntake extends SubsystemBase {
	private TalonFX reefPivotMotor;
    private SparkMax reefIntakeMotor;

    private SparkMaxConfig intakeConfig;

    private DutyCycleEncoder encoder;

    private PIDController pivotPID;
    private ArmFeedforward armFeedforward; 

    private final double lowerLimit = ReefIntakeConstants.reefRest; // needs to be fine tuned
    private final double upperLimit = ReefIntakeConstants.reefGrab; //limits for intake positions 
    
    private double setPosition;
    private double difference;

    //Reef Intake is 2.8 lbs
    //Bar is 20 inches long 

    public ReefIntake(){
        var talonFXConfigs = new TalonFXConfiguration();

        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kP = 0.01;

        intakeConfig = new SparkMaxConfig();

        intakeConfig.idleMode(IdleMode.kBrake);

        intakeConfig.encoder.positionConversionFactor(1000);
        intakeConfig.encoder.velocityConversionFactor(1000);

        intakeConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1.0,0.0,0.0);

        encoder = new DutyCycleEncoder(Constants.Ports.reefHexPort,1,Constants.ReefIntakeConstants.encoderOffset);
        
        reefPivotMotor = new TalonFX(Constants.Ports.reefPivotMotor);
        reefIntakeMotor = new SparkMax(Constants.Ports.reefIntakeMotor, MotorType.kBrushless);

        reefIntakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pivotPID = new PIDController(ReefIntakeConstants.kP, ReefIntakeConstants.kI,ReefIntakeConstants.kD);
        armFeedforward = new ArmFeedforward(ReefIntakeConstants.kS, ReefIntakeConstants.kG, ReefIntakeConstants.kV);
        
        reefPivotMotor.getConfigurator().apply(talonFXConfigs);
        reefPivotMotor.setNeutralMode(NeutralModeValue.Brake);
        //reefPivotMotor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), null, null);
        //reefPivotMotor.getConfigurator().apply(talonFXConfigs);
        //reefPivotMotor.setNeutralMode(NeutralModeValue.Brake);

        setIntakePos(ReefIntakeConstants.reefRest); //uncomment tis
    }

  /**Sets intake to suck in */
    public void sucker() {
        reefIntakeMotor.setVoltage(0.2*12);//0.3
       
    }

    /**Runs intake backwards at 0.12 speed*/
    public void outTake() {
        reefIntakeMotor.setVoltage(-2.4);
       
    }

    /**Stops the intake. */
    public void stopTake(){
        reefIntakeMotor.set(0);
        
    }   
    
    public void setVoltageIntake(double volts){
        reefIntakeMotor.setVoltage(volts);
        
    }

    public void setVoltagePivot(double volts){
        reefPivotMotor.setVoltage(volts);
        System.out.println("Volts " + volts);
    }

    public void setIntakePos(double position) {
        position = MathUtil.clamp(position, lowerLimit, upperLimit);
        
        
        setPosition = position;
        //Remember to put negative infront of pivot if the arm goes the opposite direction of the set point
        reefPivotMotor.setVoltage(-pivotPID.calculate(getPosition(), position)); //reefPivotMotor.setVoltage(-pivotPID.calculate(getPosition(), position));
    }

    /**Stops all motors */
    public void stopAll(){
        reefIntakeMotor.set(0);

        reefPivotMotor.set(0);

    }
    
    /**Returns the velocity of the intake motor. */
    public double getreefGrabSpeed(){
        return reefIntakeMotor.getEncoder().getVelocity();
    }

    /**Returns the voltage of the intake motor. */
    public double getreefGrabVoltage(){
        return reefIntakeMotor.getBusVoltage();
    }

    /**Returns the current of the reef intake motor. */
    public double getreefGrabCurrent(){
        return reefIntakeMotor.getOutputCurrent();
    }

    public double getPivotSpeed(){
        return reefPivotMotor.getVelocity().getValueAsDouble();
    }

    public double getPivotVoltage(){
        return reefPivotMotor.getMotorVoltage().getValueAsDouble();
    }

    public double getPivotCurrent(){
        return reefPivotMotor.getSupplyCurrent().getValueAsDouble();
    }

    /**Returns true of the intake is on. */
    public boolean isOn(){
        return Math.abs(getreefGrabSpeed()) > 0;
    }
        /**Returns the intake motor's rotor velocity in rotations per minute */
    public double getRPM(){
        return -(60 * reefIntakeMotor.getEncoder().getVelocity());
    }
    public double getSetPosition(){
        return setPosition;
    }

    /**Gets the position of the arm from the hex encoder */
    /*  We are zeroing when reef intake is straight up (Math.PI/2) 
     * so just subtract 0.25 (cuz this is rotations) to get the position that the ArmFeedForward expects.
     * We mutliply by 6.28 later to get the radians for the ArmFeedForward.
     */
    public double getPosition(){
        return encoder.get(); //-0.25; sign depends on which way is forward
    }
    // checks if the reef intake is ready to intake based on the position its in.     
    public boolean isReady(){
        return difference < 0.0025 || (getSetPosition() == ReefIntakeConstants.reefGrab && getPosition() < ReefIntakeConstants.reefGrab);
    }

    @Override   
    public void periodic() {

        difference = Math.abs(setPosition - getPosition());

        // SmartDashboard.putBoolean("Ball Ate detected:", ateBall());
        // SmartDashboard.putBoolean("Ball detected:", hasBall());
        SmartDashboard.putNumber("reef Intake Motor Speed", getreefGrabSpeed());
        SmartDashboard.putNumber("reef IntakeM Voltage", getreefGrabVoltage());
        SmartDashboard.putBoolean("reef Detected Color", isOn());
        SmartDashboard.putNumber("reef RPM", getRPM());
        SmartDashboard.putNumber("reef SetPosition", getSetPosition());
        SmartDashboard.putNumber("reef IntakePosition", getPosition());
        SmartDashboard.putNumber("reef Pivot Speed", getPivotSpeed());
        SmartDashboard.putNumber("reef Pivot Voltage", getPivotVoltage());
        SmartDashboard.putNumber("reef Pivot Current", getPivotCurrent());
        SmartDashboard.putBoolean("reef Pivot Ready", isReady());
        SmartDashboard.putData("reef Pivot PID", pivotPID);
        //SmartDashboard.putNumber("reef FeedForward", armFeedforward.calculate(getPosition() * 6.28,0));
        SmartDashboard.putNumber("reef PID Value", pivotPID.calculate(getPosition(), getSetPosition()));
        SmartDashboard.putNumber("reef Both", pivotPID.calculate(getPosition(), getSetPosition()) + armFeedforward.calculate(getPosition() * 6.28,0));

        setIntakePos(getSetPosition());

}
}
