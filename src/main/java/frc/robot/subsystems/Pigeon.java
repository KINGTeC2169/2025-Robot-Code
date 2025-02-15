
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;

public class Pigeon {
    
    private static Pigeon2 pigeon = new Pigeon2(Constants.Ports.pigeon);
    private ShuffleboardTab tab = Shuffleboard.getTab("Pigeon");

    public Pigeon(){
        tab.addDouble("Yaw", () -> getYaw());
        tab.addDouble("Pitch", () -> getPitch());
        tab.addDouble("Roll", () -> getRoll());
    }

    /**Returns Yaw/X. */
    public static double getX(){
        return pigeon.getYaw().getValueAsDouble();
    }

    /**Returns Pitch/Y. */
    public static double getY(){
        return pigeon.getPitch().getValueAsDouble();
    }

    /**Returns the roll. */
    public static double getRoll(){
        return pigeon.getRoll().getValueAsDouble();
    }

    /**Returns Pitch/Y. */
    public static double getPitch(){
        return pigeon.getPitch().getValueAsDouble();
    }

    /**Returns Yaw/X. */
    public static double getYaw(){
        return pigeon.getYaw().getValueAsDouble();
    }

    /**Returns the rotation2d. */
    public static Rotation2d getRotation2d(){
        return pigeon.getRotation2d();
    }

    /**Resets the pigeon. */
    public static void reset(){
        pigeon.reset();
    }

    /**Reconfigures the pigeon. */
    public static void configure(){
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
    }
}