package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import frc.robot.utils.Vector3;

public class Pigeon {

    private static Pigeon2 m_pigeon;

    public static void init() {
        m_pigeon = new Pigeon2(0);
        m_pigeon.configFactoryDefault();
    }

    public static void zero(){
        m_pigeon.setYaw(90);
    }

    public static void setYaw(double deg) {
        m_pigeon.setYaw(deg);
    }

    // Local to the robot, not the world
    // Pitch, rotates around the X, left to right, axis
    // Tilts forward and backward
    public static double getPitchRad() {
        return Math.PI * m_pigeon.getPitch() / 180;
    }

    // Local to the robot, not the world
    // Yaw, rotates around the Y, up and down, axis
    public static double getRotationRad() {
        return Math.PI * m_pigeon.getYaw() / 180;
    }

    // Local to the robot, not the world
    // Roll, rotates around the Z, forward and backward, axis
    // Tilts left and right
    public static double getRollRad() {
        return Math.PI * m_pigeon.getRoll() / 180;
    }

    // Returns the a unit vector in the direction of the z-axis relative to the pigeon
    // This vector is in reference to the basis vectors of the pigeon at the start of the competition
    public static Vector3 findKHat(){

        double yaw = getRotationRad();
        double roll = getRollRad();
        double pitch = getPitchRad();

        double x,y,z;

        x = Math.cos(yaw) * Math.sin(pitch) * Math.cos(roll) + (Math.sin(yaw) * Math.sin(roll));
        y = Math.cos(yaw) * Math.sin(roll) * -1  + (Math.sin(yaw) * Math.sin(pitch) * Math.cos(roll));
        z = Math.cos(pitch) * Math.cos(roll);
        return new Vector3(x,y,z);
        
    }
}