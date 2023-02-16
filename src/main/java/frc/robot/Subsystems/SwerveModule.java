package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    private WPI_VictorSPX driveMotor;
    private WPI_TalonSRX angleMotor;
    private PIDController angleController;
    public double xOffset;
    public double yOffset;
    private double setpoint;
    
    public SwerveModule(int driveMotorId, int angleMotorId, double initialAngle) {
        this.driveMotor = new WPI_VictorSPX(driveMotorId);
        this.angleMotor = new WPI_TalonSRX(angleMotorId);
        this.angleController = new PIDController(1, 0, 0);
        xOffset = Units.inchesToMeters(6.0);
        yOffset = Units.inchesToMeters(6.0);
        setpoint = initialAngle;

        // Configure drive motor
        driveMotor.configFactoryDefault();
        driveMotor.setInverted(false);

        // Configure angle motor
        angleMotor.configFactoryDefault();
        angleMotor.setInverted(false);
        angleMotor.setSensorPhase(false);

        // Configure PID controller
        angleController.setTolerance(1.0);
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Set initial angle of the steer motor
        setAngle(initialAngle);
    }

    public double getXOffset() {
        return xOffset;
    }
    
    public double getYOffset() {
        return yOffset;
    }
    
    public void setSpeed(double speed) {
        driveMotor.set(VictorSPXControlMode.MotionMagic, speed);
    }
    
    public void setAngle(double angle) {
        setpoint = angle;
    }
    
    public void update() {
        double error = setpoint - getAngle();
        double output = angleController.calculate(getAngle(), setpoint);
        angleMotor.set(TalonSRXControlMode.MotionMagic, output);
    }

    public double getAngle() {
        double angle = angleMotor.getSelectedSensorPosition() * 2.0 * Math.PI / 4096.0;
        return angle;
    }
    
    public void updateDashboard(String name) {
        SmartDashboard.putNumber(name + " Angle", getAngle());
        SmartDashboard.putNumber(name + " Setpoint", setpoint);
        SmartDashboard.putNumber(name + " Error", setpoint - getAngle());
    }
}
