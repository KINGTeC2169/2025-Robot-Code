package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants;

public class LED {
    private static SerialPort arduino = new SerialPort(9600, Constants.Ports.arduino, 0);

    public static void intialize(){
        arduino.writeString("A");
    }

    public static void readyToShoot(){
        arduino.writeString("B");
    }

    public static void justShot(){
        arduino.writeString("C");
    }
    
    public static void ballIn(){
    arduino.writeString("D");
    }

    public static void noBall(){
        arduino.writeString("E");
    }

    public static void off(){
        arduino.writeString("F");
    }


    
    
}
