package frc.robot.Subsystems;

import frc.robot.Utilities.Vector2;

public class SwerveManager {

    public static SwerveModule[] swerveMods;

    public static void init() {

        swerveMods = new SwerveModule[] {
            new SwerveModule(8, 1),
            new SwerveModule(2, 7),
            new SwerveModule(4, 3),
            new SwerveModule(6, 5),
        };


    }

    public static void rotateAndDrive(double rotSpeed, Vector2 move) {

        double heading = Pigeon.getRotationRad();
        
        // Array containing the unclamped movement vectors of each module
        Vector2[] vectors = new Vector2[swerveMods.length];

        // Multiply the movement vector by a rotation matrix to compensate for the pigeon's heading
        Vector2 relMove = move.rotate(-(heading - Math.PI / 2));

        // The greatest magnitude of any module's distance from the center of rotation
        double maxModPosMagnitude = 0;
        for (int i = 0; i < swerveMods.length; i++) {
            maxModPosMagnitude = Math.max(maxModPosMagnitude,
            swerveMods[i].m_pos.mag());
        }

        // The greatest speed of any of the modules. If any one module's speed is
        // greater than 1.0, all of the speeds are scaled down.
        double maxSpeed = 1.0;

        // Calculate unclamped movement vectors
        for (int i = 0; i < swerveMods.length; i++) {
            // The vector representing the direction the module should move to achieve the
            // desired rotation. Calculated by taking the derivative of the module's
            // position on the circle around the center of rotation, normalizing the
            // resulting vector according to maxModPosMagnitude (such that the magnitude of
            // the largest vector is 1), and scaling it by a factor of rotSpeed.
            
            Vector2 rotate = new Vector2(
                (-1 * swerveMods[i].m_pos.y / maxModPosMagnitude) * rotSpeed, 
                (     swerveMods[i].m_pos.x / maxModPosMagnitude) * rotSpeed);

            // The final movement vector, calculated by summing movement and rotation
            // vectors
            Vector2 rotMove = relMove.add(rotate);

            vectors[i] = rotMove;
            maxSpeed = Math.max(maxSpeed, rotMove.mag());
        }

        for (int i = 0; i < swerveMods.length; i++) {
            // Convert the movement vectors to a directions and magnitudes, clamping the
            // magnitudes based on 'max'. 
            double direction = vectors[i].atan();
            double power = vectors[i].mag() / maxSpeed;

            // Drive the swerve modules
            if(power != 0)
            swerveMods[i].rotateToRad(direction);
                swerveMods[i].drive(power);
        }


    }
}
