package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Joystick;
import frc.controllermaps.LogitechF310;
import frc.robot.Utilities.RMath;
import frc.robot.Utilities.Vector2;

public class OI {
    static Joystick driverStick;
    
    static final int moveX         = LogitechF310.AXIS_LEFT_X;
    static final int moveY         = LogitechF310.AXIS_LEFT_Y;
    static final int rotateX       = LogitechF310.AXIS_RIGHT_X;
    static final int boost         = LogitechF310.AXIS_LEFT_TRIGGER;
    static final int pigeonZero    = LogitechF310.BUTTON_Y;
    static final int driveToPoint  = LogitechF310.BUTTON_A;
    public static double x;
    public static double y;
    public static double rotation;

    static boolean drivingToPoint;

    public static void init() {
        driverStick = new Joystick(0);
    }

    public static void joystickInput() {

        // Fetch double values from the controller
        x = driverStick.getRawAxis(moveX);
        y = -driverStick.getRawAxis(moveY);
        rotation = driverStick.getRawAxis(rotateX);

        // Swerving and a steering! Zoom!
        SwerveManager.drive(x, y, rotation);

    }
}