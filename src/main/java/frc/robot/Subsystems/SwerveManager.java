package frc.robot.Subsystems;

import frc.robot.Utilities.Vector2;

public class SwerveManager {

    public static SwerveModule[] modules;
  
    public static void init() {

        modules = new SwerveModule[] {
            new SwerveModule(1, 8, 0),
            new SwerveModule(7, 2, 0),
            new SwerveModule(3, 4, 0),
            new SwerveModule(5, 6, 0),
        };

    }
  
    public static void drive(double x, double y, double rotation) {
        // Convert x, y, and rotation to polar coordinates
        double r = Math.sqrt(x * x + y * y);
        double theta = Math.atan2(y, x);
        double phi = Math.atan2(modules[0].getYOffset(), modules[0].getXOffset());
        double[] desiredAngles = new double[modules.length];
        double[] speeds = new double[modules.length];

        // Calculate the desired angle and speed for each module
        for (int i = 0; i < modules.length; i++) {
            double angle = phi + Math.atan2(modules[i].getYOffset(), modules[i].getXOffset());
            double dx = r * Math.cos(theta - angle);
            double dy = r * Math.sin(theta - angle);
            double dr = Math.sqrt(dx * dx + dy * dy);
            double speed = dr;
            desiredAngles[i] = angle;
            speeds[i] = speed;
        }

        double maxSpeed = 0.0;

        // Find the maximum speed
        for (double speed : speeds) {
            if (speed > maxSpeed) {
                maxSpeed = speed;
            }
        }

        // Set the angle and speed of each module
        for (int i = 0; i < modules.length; i++) {
            double speed = speeds[i];
            double angle = desiredAngles[i];

            // Scale down the speed if it's greater than the maximum
            if (maxSpeed > 1.0) {
                speed /= maxSpeed;
            }

            // Set the angle and speed of the module
            modules[i].setAngle(angle);
            modules[i].setSpeed(speed);
        }
    }

    public void updateDashboard() {
        for (int i = 0; i < modules.length; i++) {
            modules[i].updateDashboard("Module " + i);
        }
    }
  }
  