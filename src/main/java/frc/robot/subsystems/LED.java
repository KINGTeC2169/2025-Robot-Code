package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase {

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;

    //Used to control each strip of LED
    private AddressableLEDBufferView m_leftSide;
    private AddressableLEDBufferView m_leftMiddle;
    private AddressableLEDBufferView m_rightSide;
    private AddressableLEDBufferView m_rightMiddle;

    private int ledLength = 205;

    // Our LED strip has a density of 60 LEDs per meter
    private final Distance kLedSpacing = Meters.of(1 / 60.0);

    //TODO: Find indexes where LED strip breaks
    private final int breakOne = 63;
    private final int breakTwo = 104;
    private final int breakThree = 161  ;

    private Color royalMaroon = new Color("#A00014");
    private Color royalRed = new Color("#C00000");
    private Color royalYellow = new Color("#FAD200");

    //Patterns
    private LEDPattern breathing = LEDPattern.gradient(GradientType.kDiscontinuous, royalYellow, royalRed).breathe(Seconds.of(2));
    private LEDPattern solidGreen = LEDPattern.solid(Color.kGreen);
    private LEDPattern solidRed = LEDPattern.solid(Color.kRed);
    private LEDPattern solidBlue = LEDPattern.solid(Color.kBlue);
    private LEDPattern off = LEDPattern.kOff;

    private Map<Number, Color> blinkingBlueMaskSteps = Map.of(0, Color.kWhite, 0.32, Color.kBlack);
    private LEDPattern blinkingBlue = solidBlue.mask(LEDPattern.steps(blinkingBlueMaskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(175)));

    private LEDPattern currentPattern = breathing;

    public LED(){
        m_led = new AddressableLED(Constants.Ports.ledPort);
        m_ledBuffer = new AddressableLEDBuffer(ledLength);
        m_led.setColorOrder(ColorOrder.kRGB);

        m_led.setLength(m_ledBuffer.getLength());

        //Creates buffer views to work with the 4 chained LED strips
        m_leftSide = m_ledBuffer.createView(0, breakOne);
        m_leftMiddle = m_ledBuffer.createView(breakOne, breakTwo).reversed();
        m_rightMiddle = m_ledBuffer.createView(breakTwo, breakThree);
        m_rightSide = m_ledBuffer.createView(breakThree, ledLength - 1).reversed();

        m_led.start();
    }

    /**
     * Sets LEDs to breathing pattern.
     * 
     */
    public void initialize(){
        currentPattern = breathing;
    }

    /**
     * Sets LEDs to solid green.
     * 
     */
    public void setGreen(){
        currentPattern = solidGreen;
    }

    /**
     * Sets LEDs to solid red.
     * 
     */
    public void setRed(){
        currentPattern = solidRed;
    }

    /**
     * Sets LEDs to solid blue.
     * 
     */
    public void setBlue(){
        currentPattern = solidBlue;
    }

        /**
     * Sets LEDs to blinking blue.
     * 
     */
    public void setBlinkingBlue(){
        currentPattern = blinkingBlue;
    }

    /**
     * Sets LEDs to off.
     * 
     */    
    public void off(){
        currentPattern = off;
    }

    /**
     * Sets all 4 buffer views to the same pattern.
     * 
     * @param pattern pattern to be set
     */
    private void setAllStrips(LEDPattern pattern){
        pattern.applyTo(m_leftSide);
        pattern.applyTo(m_leftMiddle);
        pattern.applyTo(m_rightMiddle);
        pattern.applyTo(m_rightSide);
    }

    @Override
    public void periodic(){
        setAllStrips(currentPattern);
        m_led.setData(m_ledBuffer);
    }
}