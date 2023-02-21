package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** 
 * Class to control the fourbar mechanism 
 */
public class Fourbar {
    
    /* Not currently utilized, may be implemented in the future
    private static final int FOURBAR_UPPER_LIMIT = 80;
    private static final int FOURBAR_LOWER_LIMIT = -40; 
    */
    
    //constant variables
    private static final int MOTOR_ID = 2; //CORRECT ID
    private static final double MANUAL_DEADZONE = 0.3;
    private static final double BOTTOM_SETPOINT = 0;
    private static final double MID_SETPOINT = 35;
    private static final double TOP_SETPOINT = 65;
    
    //Motor and controller
    private CANSparkMax fourbarMotor;
    private RelativeEncoder relativeEncoder;
    private SparkMaxPIDController pidController;

    //variables
    private double targetSetpoint;
    private boolean menuPressed;
    private boolean rbPressed;
    private ControlMode controlMode;
    private Setpoint setpoint;
    
    //PID constants
    private static final double P = 0.1;
    private static final double I = 0.0;
    private static final double D = 1;
    private static final double F = 0.0;
    
    //self-initializes the class
    private static Fourbar instance = new Fourbar();
    
    /**
    * Sets up fourbar motor and Xbox controller, configures PID
    */
    public Fourbar(){
        fourbarMotor = new CANSparkMax(MOTOR_ID, MotorType.kBrushless);
        relativeEncoder = fourbarMotor.getEncoder();
        pidController = fourbarMotor.getPIDController();
        fourbarMotor.setIdleMode(IdleMode.kBrake);
        
        pidController.setP(P);
        pidController.setI(I);
        pidController.setD(D);
        pidController.setFF(F);
        pidController.setOutputRange(-0.5, 0.5);
        
        //Configures motor to brake when not being used
        fourbarMotor.setIdleMode(IdleMode.kBrake);
    }
    
    private enum ControlMode {
        MANUAL, PID
    }
    
    private enum Setpoint {
        BOTTOM, MID, TOP
    }
    
    /**
    * The method to be run in teleopInit to reset variables
    */
    public static void init() {
        instance.controlMode = ControlMode.MANUAL;
        instance.setpoint = Setpoint.BOTTOM;
        instance.targetSetpoint = 0;
    }
    
    /**
    * The method that will be run from teleopPeriodic
    */
    public static void run(double speed, boolean leftBumperPressed, boolean menuButtonPressed, boolean rightStickPressed) {
        
        //sets current encoder position to 0 if menu button is pressed
        setEncoderZero(menuButtonPressed);

        cycleTargetSetpoint(rightStickPressed);

        //switches between control modes
        if(leftBumperPressed) {
            if(instance.controlMode == ControlMode.MANUAL) {
                instance.controlMode = ControlMode.PID;
            } else if(instance.controlMode == ControlMode.PID) {
                instance.controlMode = ControlMode.MANUAL;
            }
        }
        
        //runs selected control mode
        if(instance.controlMode == ControlMode.MANUAL) {
            manualControl(speed);
        } else if(instance.controlMode == ControlMode.PID) {
            pidControl();
        }
        
        //prints variables to Smart Dashboard
        updateSmartDashboard(speed);
    }
    
    /**
    * Allows for cycling between setpoints using PID
    */
    private static void pidControl(){
        //set elevator PID position to target setpoint
        instance.pidController.setReference(instance.targetSetpoint, CANSparkMax.ControlType.kPosition);
    }
    
    /**
     * Allows for manual control of motor output using the right joystick
     */
    private static void manualControl(double speed){
        if(Math.abs(speed) < MANUAL_DEADZONE) { //if joystick is inside of deadzone
            speed = 0;
        }

        instance.fourbarMotor.set(-speed/3);
    }
    
    /**
     * Cycles through each setpoint
     */
    private static void cycleTargetSetpoint(boolean rightStickPressed) {

        //set setpoint enum
        if(rightStickPressed) {
            if(instance.setpoint == Setpoint.BOTTOM) {
                instance.setpoint = Setpoint.MID;
            } else if(instance.setpoint == Setpoint.MID) {
                instance.setpoint = Setpoint.TOP;
            } else if(instance.setpoint == Setpoint.TOP) {
                instance.setpoint = Setpoint.BOTTOM;
            }
        }
        
        //set targetSetpoint variable
        if(instance.setpoint == Setpoint.BOTTOM) {
            instance.targetSetpoint = BOTTOM_SETPOINT;
        } else if(instance.setpoint == Setpoint.MID) {
            instance.targetSetpoint = MID_SETPOINT;
        } else if(instance.setpoint == Setpoint.TOP) {
            instance.targetSetpoint = TOP_SETPOINT;
        }
    }

    /**
     * Sets the relative encoder position to zero if menu button is pressed
     */
    private static void setEncoderZero(boolean menuButtonPressed) {
        if(menuButtonPressed) {
            instance.relativeEncoder.setPosition(0);
        }
    }

    /**
     * Prints variables to Smart Dashboard
     */
    private static void updateSmartDashboard(double speed) {
        SmartDashboard.putNumber("FB Speed", speed);
        SmartDashboard.putString("FB Setpoint", instance.setpoint.toString());
        SmartDashboard.putString("FB Control Mode", instance.controlMode.toString());
        SmartDashboard.putNumber("FB Position", instance.relativeEncoder.getPosition());
        SmartDashboard.putNumber("FB Target Setpoint", instance.targetSetpoint);
        SmartDashboard.putNumber("FB Motor Power", instance.fourbarMotor.get()); //doesn't update correctly, fix later
    }
}