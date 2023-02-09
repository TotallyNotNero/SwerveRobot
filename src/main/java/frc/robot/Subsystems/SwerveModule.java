package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.Utilities.Vector2;

public class SwerveModule {
    
    // Our Drive motors use VictorSPXs, and our Steer gearboxes use TalonSRXs.
    // (Talons have a dedicated port for encoders - steer gearboxes need encoders).
    // IDs for the Talons are even, while the Victor IDs are odd.
    public VictorSPX driveMotor;
    public TalonSRX steerMotor;

    public Vector2 m_pos;

    private boolean inverted;

    private static final double ticksPerRotationSteer = 2048 * 12.8;
    private static final double ticksPerRotationDrive = 2048 * 8.14;

    // PID: 5,0,0.

    public SwerveModule(int dID, int sID) {
        this.driveMotor = new VictorSPX(dID);
        this.steerMotor = new TalonSRX(sID);

        this.steerMotor.configFactoryDefault();
        this.steerMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        this.steerMotor.config_kP(0, 0.2, 0);
        this.steerMotor.config_kI(0, 0.01, 0);
        this.steerMotor.config_kD(0, 0, 0);

        this.driveMotor.configFactoryDefault();
        this.driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        this.driveMotor.config_kP(0, 0.09, 0);
        this.driveMotor.config_kI(0, 0.01, 0);
        this.driveMotor.config_kD(0, 0, 0);

        inverted = false;
    }

    public void drive(double power) {
        driveMotor.set(VictorSPXControlMode.PercentOutput, power * (inverted ? -1.0 : 1.0));
    }

    public void rotateToRad(double angle) {
        rotate((angle - Math.PI * 0.5) / (2 * Math.PI) * ticksPerRotationSteer);
    }

    public void rotate(double toAngle) {
        double motorPos = steerMotor.getSelectedSensorPosition();

        // The number of full rotations the motor has made
        int numRot = (int) Math.floor(motorPos / ticksPerRotationSteer);

        // The target motor position dictated by the joystick, in motor ticks
        double joystickTarget = numRot * ticksPerRotationSteer + toAngle;
        double joystickTargetPlus = joystickTarget + ticksPerRotationSteer;
        double joystickTargetMinus = joystickTarget - ticksPerRotationSteer;

        // The true destination for the motor to rotate to
        double destination;

        // Determine if, based on the current motor position, it should stay in the same
        // rotation, enter the next, or return to the previous.
        if (Math.abs(joystickTarget - motorPos) < Math.abs(joystickTargetPlus - motorPos)
                && Math.abs(joystickTarget - motorPos) < Math.abs(joystickTargetMinus - motorPos)) {
            destination = joystickTarget;
        } else if (Math.abs(joystickTargetPlus - motorPos) < Math.abs(joystickTargetMinus - motorPos)) {
            destination = joystickTargetPlus;
        } else {
            destination = joystickTargetMinus;
        }

        // If the target position is farther than a quarter rotation away from the
        // current position, invert its direction instead of rotating it the full
        // distance
        if (Math.abs(destination - motorPos) > ticksPerRotationSteer / 4.0) {
            inverted = true;
            if (destination > motorPos)
                destination -= ticksPerRotationSteer / 2.0;
            else
                destination += ticksPerRotationSteer / 2.0;
        } else {
            inverted = false;
        }

        steerMotor.set(TalonSRXControlMode.MotionMagic, destination);

        
    }

}
