package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.systems.Intake;
import frc.robot.systems.ElevFourbar.ControlType;
import frc.robot.systems.ElevFourbar.Setpoint;
import frc.robot.systems.Intake.SolenoidState;

/** 
 * Class to control the fourbar mechanism 
 */
public class Fourbar {
    //Not currently utilized, may be implemented in the future
    private static final int FOURBAR_UPPER_LIMIT = 0;
    private static final int FOURBAR_LOWER_LIMIT = 90; 
    
    private static final double FOURBAR_SPEED_UP = -0.4;
    private static final double FOURBAR_SPEED_DOWN = 0.2;

    private static final double ENCODER_OFFSET = 0.332 * 360;
    
    //constant variables
    private static final int MOTOR_ID = 62; //CORRECT ID
    private static final double MANUAL_DEADZONE = 0.3;

    //position constants, in degrees
    private static final int SUBSTATION_INTAKE_SETPOINT = 33 ;
    private static final int GROUND_INTAKE_UP_SETPOINT = 140;
    private static final int GROUND_INTAKE_DOWN_SETPOINT = 112;
    private static final int MID_SCORING_SETPOINT = 33;
    private static final int HIGH_SCORING_SETPOINT = 73;
    private static final int STOWED_SETPOINT = 0;

    //PID constants
    private static final double P = 10;
    private static final double I = 0.0;
    private static final double D = 1;
    private static final double F = 0.0;

    //self-initializes the class
    private static final Fourbar instance = new Fourbar();
                                                      
    //Motor and controller
    private final CANSparkMax fourbarMotor;   
    private final SparkMaxPIDController pidController; 
    private SparkMaxAbsoluteEncoder absoluteEncoder;
    private RelativeEncoder relativeEncoder;

    //variables
    private double targetSetpoint;
    private double encoderPosition;
    
    /**
     * Sets up fourbar motor and Xbox controller, configures PID
     */
    public Fourbar(){
        fourbarMotor = new CANSparkMax(MOTOR_ID, MotorType.kBrushless);

        relativeEncoder = fourbarMotor.getEncoder();
        absoluteEncoder = fourbarMotor.getAbsoluteEncoder(Type.kDutyCycle);
        pidController = fourbarMotor.getPIDController();
        
        pidController.setP(P);
        pidController.setI(I);
        pidController.setD(D);
        pidController.setFF(F);
        pidController.setOutputRange(FOURBAR_SPEED_UP, FOURBAR_SPEED_DOWN);

        pidController.setFeedbackDevice(absoluteEncoder);
        pidController.setPositionPIDWrappingEnabled(true);
        pidController.setPositionPIDWrappingMaxInput(360);
        pidController.setPositionPIDWrappingMinInput(0);

        fourbarMotor.setInverted(true);
        fourbarMotor.setIdleMode(IdleMode.kBrake);

        relativeEncoder.setPositionConversionFactor(1);
    }
    
    /**
     * The method that will be run from teleopPeriodic
     * @param speed - The speed the motor will run at
     * @param switchControlMode - toggles between manual and position control
     * @param setPositionZero - sets the current encoder position to zero
     * @param changeSetpoint - cycles through the top, mid, and bottom setpoints
     */
    public static void run(double speed, boolean setPositionZero, Setpoint setpoint, ControlType controlType) {
        instance.encoderPosition = (instance.absoluteEncoder.getPosition()*360) - ENCODER_OFFSET;

        updateTargetSetpoint(setpoint);

        //sets current encoder position to 0 if menu button is pressed
        if(setPositionZero) {
            instance.relativeEncoder.setPosition(0);
        }
        
        //runs selected control mode
        if(controlType == ControlType.MANUAL) {
            manualControl(speed);
        } else if(controlType == ControlType.POSITION) {
            pidControl();
        }
        
        //prints variables to Smart Dashboard
        updateSmartDashboard(speed);
    }

    public static void autonomousRun(Setpoint setpoint) {
        instance.encoderPosition = instance.relativeEncoder.getPosition() * 360;

        updateTargetSetpoint(setpoint);

        pidControl();
    }
    
    /**
     * Allows for cycling between setpoints using PID
     */
    private static void pidControl(){
        /* //set elevator PID position to target setpoint
        if(instance.encoderPosition > 200) {
            instance.pidController.setOutputRange(-0.0, FOURBAR_SPEED_DOWN);
            instance.fourbarMotor.set(0.1);
        } else {
            instance.pidController.setOutputRange(FOURBAR_SPEED_UP, FOURBAR_SPEED_DOWN);
        } */

        instance.pidController.setReference((instance.targetSetpoint + ENCODER_OFFSET) / 360.0, CANSparkMax.ControlType.kPosition);
    }
    
    /**
     * Allows for manual control of motor output using the right joystick
     */
    private static void manualControl(double speed){
        /* if (Math.abs(speed) < MANUAL_DEADZONE) { //if joystick is inside of deadzone
            speed = 0;
        } */

        if(instance.encoderPosition >= 200) {
            speed = 0.1;
        } /* else if (instance.encoderPosition <= FOURBAR_UPPER_LIMIT/360.0 && speed >= 0) {
            speed = 0;
        } */
        double gravitybias = 0.07*Math.sin(instance.encoderPosition*3.14159/180.0);
        double outputspeed = speed * 0.4-gravitybias;
        instance.fourbarMotor.set(outputspeed);
    }

    private static void updateTargetSetpoint(Setpoint setpoint) {
        switch (setpoint) {
            case SUBSTATION_INTAKE:
                instance.targetSetpoint = SUBSTATION_INTAKE_SETPOINT;
                break;
            case GROUND_INTAKE:
                if(Intake.getPivotState() == SolenoidState.UP) {
                    instance.targetSetpoint = GROUND_INTAKE_UP_SETPOINT;
                } else{
                    instance.targetSetpoint = GROUND_INTAKE_DOWN_SETPOINT;
                }
                break;
            case MID_SCORING:
                instance.targetSetpoint = MID_SCORING_SETPOINT;
                break;
            case HIGH_SCORING:
                instance.targetSetpoint = HIGH_SCORING_SETPOINT;
                break;
            case STOWED:
                instance.targetSetpoint = STOWED_SETPOINT;
                break;
        }
    }

    public static double getPosition() {
        return instance.encoderPosition;
    }

    public static double getTargetSetpoint() {
        return instance.targetSetpoint;
    }

    public static boolean getIsFinished() {
        return Math.abs(instance.encoderPosition - instance.targetSetpoint) < 1;
    }

    /**
     * Prints variables to Smart Dashboard
     */
    private static void updateSmartDashboard(double speed) {
        SmartDashboard.putNumber("FB Speed", speed);
        SmartDashboard.putNumber("FB Position", instance.encoderPosition);
        SmartDashboard.putNumber("FB Target Setpoint", instance.targetSetpoint);
        SmartDashboard.putNumber("FB Motor Power", instance.fourbarMotor.getAppliedOutput()); //doesn't update correctly, fix later
        SmartDashboard.putNumber("OutputRange", instance.pidController.getOutputMax());
    }
}