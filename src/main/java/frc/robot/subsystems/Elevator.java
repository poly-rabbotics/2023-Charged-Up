package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Controls the elevator
 */
public class Elevator {

    private static final int TICKS_PER_INCH = -23890; //FINAL VAlUE DO NOT CHANGE
    
    //constant variables
    private static final double MANUAL_DEADZONE = 0.3;
    private static final int ELEVATOR_MOTOR_ID = 5; //CORRECT ID
    private static final int ELEVATOR_BOTTOM_SETPOINT = 0;
    private static final int ELEVATOR_MID_SETPOINT = TICKS_PER_INCH * 16; 
    private static final int ELEVATOR_TOP_SETPOINT = TICKS_PER_INCH * 28; 
    
    //Motor and controller
    TalonFX elevatorMotor;
    DigitalInput bottomLimitSwitch;

    //variables
    private double encoderPosition;
    private double overShoot;
    private int targetSetpoint;
    private boolean rbPressed = false;
    private ElevatorMode controlMode;
    private ElevatorSetpoint setpoint;

    //PID constants
    private final double P = 1.0;
    private final double I = 0.00001;
    private final double D = 0.01;
    
    //self-initializes the class
    private static Elevator instance = new Elevator();
    
    /**
     * Sets up elevator motor and Xbox controller, configures PID
     */
    private Elevator() {
        elevatorMotor = new TalonFX(ELEVATOR_MOTOR_ID);
        bottomLimitSwitch = new DigitalInput(0);

        elevatorMotor.configFactoryDefault();
        elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
        elevatorMotor.config_kP(0, P);
        elevatorMotor.config_kI(0, I);
        elevatorMotor.config_kD(0, D);
        elevatorMotor.selectProfileSlot(0, 0);
        
        //Configures motor to brake when not being used
        elevatorMotor.setNeutralMode(NeutralMode.Brake);
        
    }

    private enum ElevatorMode {
        MANUAL, POSITION
    }

    private enum ElevatorSetpoint {
        BOTTOM, MID, TOP
    }

    /**
     * The method to be run in teleopInit to reset variables
     */
    public static void init() {
        //resetting these in init makes it so the robot does not automatically go into position control when enabling
        instance.controlMode = ElevatorMode.MANUAL;
        instance.setpoint = ElevatorSetpoint.BOTTOM;
    }
    
    /**
    * The method that will be run in teleopPeriodic
    */
    public static void run(double speed, boolean rightBumperPressed, boolean startButtonPressed, boolean aButtonPressed, boolean bButtonPressed, boolean yButtonPressed, int dPadDirection) {
        instance.encoderPosition = instance.elevatorMotor.getSensorCollection().getIntegratedSensorPosition();

        //sets current encoder position to 0 if start button is pressed
        setEncoderZero(startButtonPressed);
        
        //switches between manual and position control modes
        if(rightBumperPressed) {
            if(instance.controlMode == ElevatorMode.POSITION) {
                instance.controlMode = ElevatorMode.MANUAL;
            } else {
                instance.controlMode = ElevatorMode.POSITION;
            }
        }

        //switches between setpoints
        updateTargetSetpoint(aButtonPressed, bButtonPressed, yButtonPressed);
        
        //runs control mode
        if(instance.controlMode == ElevatorMode.MANUAL) { //manual control
            manualControl(speed, dPadDirection);
        } if(instance.controlMode == ElevatorMode.POSITION) { //position control
            positionControl();
        }

        //prints variables to Smart Dashboard
        updateSmartDashboard(speed, dPadDirection);
    }

    /**
     * Manual control of the elevator using the left joystick
     */
    private static void manualControl(double speed, int dPadDirection) {

        if(Math.abs(speed) < MANUAL_DEADZONE || (!instance.bottomLimitSwitch.get() && speed > MANUAL_DEADZONE)) {
            speed = 0;
        } 
        
        //low sensitivity control using DPAD
        if(dPadDirection == 180) {
            speed = 0.1;
        } else if(dPadDirection == 0) {
            speed = -0.1;
        }

        instance.elevatorMotor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * PID Control of the elevator, cycles through
     * setpoints bottom, mid, top
     */
    private static void positionControl() {
        
        //calculates the overshoot in encoder ticks, used for tuning PID
        if(Math.abs(instance.encoderPosition) - Math.abs(instance.targetSetpoint)  > instance.overShoot && Math.abs(instance.encoderPosition) > Math.abs(instance.targetSetpoint)) {
            instance.overShoot = Math.abs(instance.encoderPosition) - Math.abs(instance.targetSetpoint);
        }

        //set elevator PID position to target setpoint
        instance.elevatorMotor.set(ControlMode.Position, instance.targetSetpoint);
    }

    /**
    * Sets the encoder position to 0 if the start button is pressed
    */
    private static void setEncoderZero(boolean startButtonPressed) {
        if(startButtonPressed || !instance.bottomLimitSwitch.get()) {
            instance.elevatorMotor.getSensorCollection().setIntegratedSensorPosition(0, 30);
        }
    }

    private static void updateTargetSetpoint(boolean aButtonPressed, boolean bButtonPressed, boolean yButtonPressed) {
        //updates the setpoint enum
        if(aButtonPressed) {
            instance.setpoint = ElevatorSetpoint.BOTTOM;
            instance.overShoot = 0;
        } else if(bButtonPressed) {
            instance.setpoint = ElevatorSetpoint.MID;
            instance.overShoot = 0;
        } else if(yButtonPressed) {
            instance.setpoint = ElevatorSetpoint.TOP;
            instance.overShoot = 0;
        }

        //updates targetSetpoint variable
        if(instance.setpoint == ElevatorSetpoint.BOTTOM) {
            instance.targetSetpoint = ELEVATOR_BOTTOM_SETPOINT;
        } else if(instance.setpoint == ElevatorSetpoint.MID) {
            instance.targetSetpoint = ELEVATOR_MID_SETPOINT;
        } else if(instance.setpoint == ElevatorSetpoint.TOP) {
            instance.targetSetpoint = ELEVATOR_TOP_SETPOINT;
        }
    }

    private static void updateSmartDashboard(double speed, int dPadDirection) {
        SmartDashboard.putString("Elev Control Mode", instance.controlMode.toString());
        SmartDashboard.putString("Elev Setpoint", instance.setpoint.toString());
        SmartDashboard.putNumber("Elev Speed", speed);
        SmartDashboard.putNumber("Elev Position", instance.encoderPosition);
        SmartDashboard.putNumber("Elev Target Position", instance.targetSetpoint);
        SmartDashboard.putNumber("Elev Overshoot", instance.overShoot);
        SmartDashboard.putNumber("Elev Motor Power", instance.elevatorMotor.getMotorOutputPercent());
        SmartDashboard.putBoolean("Limit Switch", !instance.bottomLimitSwitch.get());
        SmartDashboard.putNumber("POV", dPadDirection);
    }
}
