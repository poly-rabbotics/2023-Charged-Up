package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.systems.Intake.SolenoidState;
import frc.robot.systems.ElevFourbar;
import frc.robot.systems.Intake;

/** 
 * Class to control the fourbar mechanism 
 */
public class Fourbar {
    
    //PID max speed values
    private static final double FOURBAR_SPEED_UP = -0.45;

    //encoder offset
    public final double ENCODER_OFFSET = 0.330 * 360;
    
    //constant variables
    private static final int MOTOR_ID = 61; //CORRECT ID
    private static final double MANUAL_DEADZONE = 0.2;

    //PID constants
    private static final double P0 = 5;    
    private static final double I0 = 0.0;
    private static final double D0 = 0.05;
    private static final double F0 = 0.0;

    private static final double P1 = 2.5;
    private static final double I1 = 0.0;
    private static final double D1 = 0.3;
    private static final double F1 = 0.0;
    
    //Motor and controller
    private final CANSparkMax fourbarMotor;   
    private final SparkPIDController pidController; 
    private SparkAbsoluteEncoder absoluteEncoder;

    //instance variables
    private double targetSetpoint;
    private double encoderPosition;
    private double bumperIntercept = 0;
    private double slope = 0;
    
    public void setBrake(boolean brake) {
        if (brake) {
            fourbarMotor.setIdleMode(IdleMode.kBrake);
        } else {
            fourbarMotor.setIdleMode(IdleMode.kCoast);
        }
    }

    /**
     * Sets up fourbar motor and Xbox controller, configures PID
     */
    public Fourbar(){
        fourbarMotor = new CANSparkMax(MOTOR_ID, MotorType.kBrushless);
        fourbarMotor.setSmartCurrentLimit(30);

        absoluteEncoder = fourbarMotor.getAbsoluteEncoder(Type.kDutyCycle);
        pidController = fourbarMotor.getPIDController();
        
        pidController.setP(P0, 0);
        pidController.setI(I0, 0);
        pidController.setD(D0, 0);
        pidController.setFF(F0, 0);

        pidController.setP(P1, 1);
        pidController.setI(I1, 1);
        pidController.setD(D1, 1);
        pidController.setFF(F1, 1); 

        pidController.setOutputRange(FOURBAR_SPEED_UP, -FOURBAR_SPEED_UP);

        pidController.setFeedbackDevice(absoluteEncoder);
        pidController.setPositionPIDWrappingEnabled(true);
        pidController.setPositionPIDWrappingMaxInput(360);
        pidController.setPositionPIDWrappingMinInput(0);

        fourbarMotor.setInverted(true);
        fourbarMotor.setIdleMode(IdleMode.kBrake);
    }
    
    /**
     * Allows for cycling between setpoints using PID
     * @param setpoint The setpoint to move to, as defined in the Setpoint enum
     */
    public void pidControl(Setpoint setpoint){
        
        pidController.setOutputRange(FOURBAR_SPEED_UP, -FOURBAR_SPEED_UP);

        encoderPosition = (absoluteEncoder.getPosition()*360) - ENCODER_OFFSET;
        
        targetSetpoint = setpoint.getFourbarPos();

        pidController.setReference((targetSetpoint + ENCODER_OFFSET) / 360.0, CANSparkMax.ControlType.kPosition, 0);
        
    }

    /**
     * Allows for manual control of motor output using the right joystick
     */
    public void manualControl(double speed){

        pidController.setOutputRange(0.1, -0.6);

        encoderPosition = (absoluteEncoder.getPosition()*360) - ENCODER_OFFSET;

        //TODO: FIX THIS
        Coordinate coords = ElevFourbar.getCurrentPos();
        double b = ElevFourbar.elevator.getPosition();
        slope = (coords.y - b) / coords.x;

        bumperIntercept = (slope * Setpoint.BUMPER_X) + b;

        if(bumperIntercept <= Setpoint.BUMPER_Y) {
            if(speed < 0) {
                speed = 0;
            }
        } else if(coords.y <= -3 && Intake.getPivotState() == SolenoidState.UP) {
            if(speed < 0) {
                speed = 0;
            }
        } else if(coords.y <= 21 && Intake.getPivotState() == SolenoidState.DOWN) {
            if(speed < 0) {
                speed = 0;
            }
        }

        //Deadzone
        if(Math.abs(speed) < MANUAL_DEADZONE) {
            speed = 0;
        }

        targetSetpoint -= speed * 0.9;

        pidController.setReference((targetSetpoint + ENCODER_OFFSET) / 360.0, CANSparkMax.ControlType.kPosition, 1);

        /* speed /= 2;
        fourbarMotor.set(-speed); */
    }

    /**
     * @return Encoder position in degrees
     */
    public double getPosition() {
        return encoderPosition;
    }

    /**
     * @return Target encoder position in degrees
     */
    public double getTargetPosition() {
        return targetSetpoint;
    }

    /**
     * @return Position of the absolute encoder from 0 to 1
     */
    public double getAbsolutePosition() {
        return absoluteEncoder.getPosition();
    }

    public void setPIDSpeed(double speed) {
        pidController.setOutputRange(-speed, speed);
        
    }

    public double getBumperIntercept() {
        return bumperIntercept;
    }

    public double getSlope() {
        return slope;
    }

    public double getPower() {
        return fourbarMotor.get();
    }
}