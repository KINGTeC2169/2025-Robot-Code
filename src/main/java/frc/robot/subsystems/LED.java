package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants;

public class LED {
    private static SerialPort arduino = new SerialPort(9600, Constants.Ports.arduino, 8, SerialPort.Parity.kNone, SerialPort.StopBits.kOne);

    public static void intialize(){
        arduino.writeString("I"); 
    }

    public static void setRed(){
        arduino.writeString("R"); //RED
    }

    public static void setYellow(){ //intakeRunning
        arduino.writeString("B"); //YELLOW
    }

    public static void setBlue(){ //intakeHasBall
        arduino.writeString("C"); //BLUE
    }
    
    public static void setPurple(){ //shootingREVing
        arduino.writeString("D"); //PURPLE
    }

    public static void setGreen(){ //shootingShot
        arduino.writeString("E"); //GREEN
    }

    public static void setBlack(){ //off
        arduino.writeString("F"); //BLACK
    }


    
    
}
