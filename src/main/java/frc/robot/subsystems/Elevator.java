package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Controls the elevator
 */
public class Elevator {
    //Ticks per inch constant, gear reduction is 12:1
    private static final double TICKS_PER_INCH = -6144.0; //FINAL VALUE DO NOT CHANGE
    
    //constant variables
    private static final double MANUAL_DEADZONE = 0.3;
    private static final int ELEVATOR_MOTOR_ID = 62; //CORRECT ID
    
    //PID constants
    private static final double P = 1.0;
    private static final double I = 0.00001;
    private static final double D = 0.01;
    
    //Motor and controller
    private final TalonFX elevatorMotor;

    //variables
    private double encoderPosition;
    private double targetSetpoint;

    private DigitalInput limit;
    
    /**
     * Sets up elevator motor and Xbox controller, configures PID
     */
    public Elevator() {
        elevatorMotor = new TalonFX(ELEVATOR_MOTOR_ID);

        elevatorMotor.configFactoryDefault();
        elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
        elevatorMotor.config_kP(0, P);
        elevatorMotor.config_kI(0, I);
        elevatorMotor.config_kD(0, D);
        elevatorMotor.selectProfileSlot(0, 0);

        elevatorMotor.configPeakOutputForward(0.7);
        elevatorMotor.configPeakOutputReverse(-0.7);
        
        //Configures motor to brake when not being used
        elevatorMotor.setNeutralMode(NeutralMode.Brake);
        limit = new DigitalInput(0);
    }

    /**
     * The method to be run in teleopInit to reset variables
     */
    public void init() {

    }

    public void autonomousInit() {
        targetSetpoint = 0;
    }

    /**
     * Manual control of the elevator using the left joystick
     * @param speed The speed of the elevator from 0 to 1
     * @param dPadDirection The value of the dpad
     */
    public void manualControl(double speed, int dPadDirection) {
        encoderPosition = elevatorMotor.getSensorCollection().getIntegratedSensorPosition();

        if (Math.abs(speed) < MANUAL_DEADZONE) {
            speed = 0;
        } 
        
        //low sensitivity control using DPAD
        if (dPadDirection == 180) {
            speed = 0.2;
        } else if (dPadDirection == 0) {
            speed = -0.2;
        }

        if (!limit.get()) {
            if (speed > 0.0) {
                speed = 0.0;
            }
        }

        elevatorMotor.set(ControlMode.PercentOutput, speed * 0.6);
    }

    /**
     * PID Control of the elevator
     * @param setpoint The setpoint to move the elevator to
     */
    public void pidControl(Setpoint setpoint) {
        encoderPosition = elevatorMotor.getSensorCollection().getIntegratedSensorPosition();

        targetSetpoint = setpoint.getElevPos();

        //set elevator PID position to target setpoint
        elevatorMotor.set(ControlMode.Position, targetSetpoint * TICKS_PER_INCH);
    }

    /**
     * 
     * @return Encoder position in inches
     */
    public double getPosition() {
        return encoderPosition / TICKS_PER_INCH;
    }

    /**
     * 
     * @return Target encoder position in inches
     */
    public double getTargetPosition() {
        return targetSetpoint;
    }

    public void zeroEncoder() {
        elevatorMotor.getSensorCollection().setIntegratedSensorPosition(2000, 30);
    }
}
