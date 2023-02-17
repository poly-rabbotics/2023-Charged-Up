package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
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
    XboxController controller; //get rid of this once merged, we need to use a universal controller

    //variables
    private double speed;
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
        controller = new XboxController(0);

        elevatorMotor.configFactoryDefault();
        elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
        elevatorMotor.config_kP(0, P);
        elevatorMotor.config_kI(0, I);
        elevatorMotor.config_kD(0, D);
        elevatorMotor.selectProfileSlot(0, 0);
        
        //Configures motor to brake when not being used
        instance.elevatorMotor.setNeutralMode(NeutralMode.Brake);
        
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
    public static void run() {
        instance.speed = getElevatorSpeed();
        instance.encoderPosition = instance.elevatorMotor.getSensorCollection().getIntegratedSensorPosition();

        //sets current encoder position to 0 if start button is pressed
        setEncoderZero();
        
        //switches between manual and position control modes
        if(getSwitchControlMode()) {
            if(instance.controlMode == ElevatorMode.POSITION) {
                instance.controlMode = ElevatorMode.MANUAL;
            } else {
                instance.controlMode = ElevatorMode.POSITION;
            }
        }

        //switches between setpoints
        updateTargetSetpoint();
        
        //runs control mode
        if(instance.controlMode == ElevatorMode.MANUAL) { //manual control
            manualControl();
        } if(instance.controlMode == ElevatorMode.POSITION) { //position control
            positionControl();
        }

        //prints variables to Smart Dashboard
        updateSmartDashboard();
    }

    /**
     * Manual control of the elevator using the left joystick
     */
    private static void manualControl() {
        if(Math.abs(instance.speed) > MANUAL_DEADZONE) {
            instance.elevatorMotor.set(ControlMode.PercentOutput, instance.speed);
        } else {
            instance.elevatorMotor.set(ControlMode.PercentOutput, 0);
        }
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
    private static void setEncoderZero() {
        if(instance.controller.getRawButton(8)) {
            instance.elevatorMotor.getSensorCollection().setIntegratedSensorPosition(0, 30);
        }
    }
    
    /**
    * Gets the left joystick's value to be used for percent output
    * @return Left joystick Y axis
    */
    private static double getElevatorSpeed() {
        return (instance.controller.getRawAxis(1));
    }
    
    /**
    * Gets the value of the A button to be used for switching control modes
    * @return True once if A button is pressed 
    */
    private static boolean getSwitchControlMode() {
        if(!instance.rbPressed && instance.controller.getRawButton(6)) {
            instance.rbPressed = true;
            return true;
        } else if(instance.rbPressed && !instance.controller.getRawButton(6)) {
            instance.rbPressed = false;
            return false;
        } else return false;
    }

    private static void updateTargetSetpoint() {

        //updates the setpoint enum
        if(instance.controller.getRawButton(1)) {//A button
            instance.setpoint = ElevatorSetpoint.BOTTOM;
            instance.overShoot = 0;
        } else if(instance.controller.getRawButton(2)) {//B button
            instance.setpoint = ElevatorSetpoint.MID;
            instance.overShoot = 0;
        } else if(instance.controller.getRawButton(4)) {//Y button
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

    private static void updateSmartDashboard() {
        SmartDashboard.putString("Elev Control Mode", instance.controlMode.toString());
        SmartDashboard.putString("Elev Setpoint", instance.setpoint.toString());
        SmartDashboard.putNumber("Elev Speed", instance.speed);
        SmartDashboard.putNumber("Elev Position", instance.encoderPosition);
        SmartDashboard.putNumber("Elev Target Position", instance.targetSetpoint);
        SmartDashboard.putNumber("Elev Overshoot", instance.overShoot);
        SmartDashboard.putNumber("Elev Motor Power", instance.elevatorMotor.getMotorOutputPercent());
    }
}
