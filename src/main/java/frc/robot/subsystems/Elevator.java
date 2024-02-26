package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Controls the elevator
 */
public class Elevator {
    //Ticks per inch constant, gear reduction is 12:1
    private static final double TICKS_PER_INCH = -6144.0; //FINAL VALUE DO NOT CHANGE
    // THROUGH BORE = -455.156
    
    //constant variables
    private static final double MANUAL_DEADZONE = 0.3;
    private static final int ELEVATOR_MOTOR_ID = 62; //CORRECT ID

    private static final int ENCODER_CHANNEL_A = 4;
    private static final int ENCODER_CHANNEL_B = 5;
    
    //PID constants
    private static final double P = 1.0;
    private static final double I = 0.00001;
    private static final double D = 0.01;
    
    //Motor and controller
    private final TalonFX elevatorMotor;

    //Encoder and CAN Spark Max
    private Encoder encoder;


    //variables
    private double encoderPosition;
    private double targetSetpoint;

    //2024 stuff I have to make this work for the APEC conference. Tomorrow. If robotics gods exist, please make this work.
    //Also praise past me for making this organized, I didn't think I'd have to work on it again but its very much easier than it wouldve been.
    Slot0Configs slot0Configs;
    PositionVoltage request;

    private DigitalInput limit;
    
    /**
     * Sets up elevator motor and Xbox controller, configures PID
     */
    public Elevator() {

        //Encoder
        encoder = new Encoder(ENCODER_CHANNEL_A, ENCODER_CHANNEL_B);
        
        elevatorMotor = new TalonFX(ELEVATOR_MOTOR_ID);
        slot0Configs = new Slot0Configs();

        slot0Configs.kP = P;
        slot0Configs.kI = I;
        slot0Configs.kD = D;
        
        //Configures motor to brake when not being used
        elevatorMotor.setNeutralMode(NeutralModeValue.Brake);
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
        encoderPosition = elevatorMotor.getPosition().getValueAsDouble();

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

        elevatorMotor.set(speed * 0.6);
    
        SmartDashboard.putNumber("Elev Encoder", encoder.get());
    }

    /**
     * PID Control of the elevator
     * @param setpoint The setpoint to move the elevator to
     */
    public void pidControl(Setpoint setpoint) {
        encoderPosition = elevatorMotor.getPosition().getValueAsDouble();

        targetSetpoint = setpoint.getElevPos();

        //set elevator PID position to target setpoint
        elevatorMotor.setControl(request.withPosition(targetSetpoint * TICKS_PER_INCH));

        if(encoder.get() >= 0) {
            zeroEncoder(0.25);
        }

        SmartDashboard.putNumber("Elev Encoder", encoder.get());
        SmartDashboard.putNumber("Elev Pos", getPosition());
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

    public void zeroEncoder(double inches) {
        double ticks = inches * TICKS_PER_INCH;

        elevatorMotor.setPosition(2000);
    }
}
