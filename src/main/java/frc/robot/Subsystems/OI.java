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


    static boolean drivingToPoint;

    public static void init() {
        driverStick = new Joystick(0);
    }

    public static void joystickInput() {

        // if(driverStick.getRawButton(driveToPoint)){
        //     drivingToPoint = true;
        //     SwervePID.setDestPt(new Vector2(0,0));
        //     SwervePID.setDestRot(Math.PI / 2);
        // }

         Vector2 drive = new Vector2(driverStick.getRawAxis(moveX),-driverStick.getRawAxis(moveY));

         if (drive.mag() < 0.125)
             drive = new Vector2(0,0);
         else
             drive = RMath.smoothJoystick2(drive);

        if(driverStick.getRawButton(pigeonZero))
            Pigeon.zero();

        if(driverStick.getRawButton(driveToPoint)){
            drivingToPoint = true;
            // Scoring.setScoringTarget(4);
        }

         double rotate = RMath.smoothJoystick1(driverStick.getRawAxis(rotateX)) * -0.3;

         if(Math.abs(rotate) < 0.005)
             rotate = 0;
        
        if(!drivingToPoint)
             SwerveManager.rotateAndDrive(rotate, drive);
         else {
            // Scoring.update();
            // if(Scoring.isDone)
                drivingToPoint = false;
        }

    }
}