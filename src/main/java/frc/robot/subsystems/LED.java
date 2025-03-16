package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants;

public class LED {
    private static SerialPort arduino = new SerialPort(9600, Constants.Ports.arduino, 8, SerialPort.Parity.kNone, SerialPort.StopBits.kOne);

    public static void intialize(){
        arduino.writeString("A"); //RED
    }

    public static void intakeRunning(){
        arduino.writeString("B"); //YELLOW
    }

    public static void intakeHasBall(){
        arduino.writeString("C"); //BLUE
    }
    
    public static void shootingREVing(){
    arduino.writeString("D"); //PURPLE
    }

    public static void shootingShot(){
        arduino.writeString("E"); //GREEN
    }

    public static void off(){
        arduino.writeString("F"); //BLACK
    }


    
    
}
