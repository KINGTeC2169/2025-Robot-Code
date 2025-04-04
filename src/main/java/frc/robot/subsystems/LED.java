package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants;

public class LED {
    // private static SerialPort arduino = new SerialPort(9600, Constants.Ports.arduino, 8, SerialPort.Parity.kNone, SerialPort.StopBits.kOne);

    public static void intialize(){
        //arduino.writeString("E"); 
    }

    public static void setRed(){
        //arduino.writeString("R"); //RED
    }

   

    public static void setYellow(){ //intakeHasBall
        //arduino.writeString("Y"); //YELLOW
    }
    
    

    public static void setGreen(){ //shootingShot
        //arduino.writeString("G"); //GREEN
    }

    public static void setBlack(){ //off
        //arduino.writeString("A"); //BLACK
    }


    
    
}
