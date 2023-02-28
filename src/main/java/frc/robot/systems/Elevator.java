package frc.robot.systems;

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
    private static final double TICKS_PER_INCH = -10752.0; //FINAL VAlUE DO NOT CHANGE
    
    //constant variables
    private static final double MANUAL_DEADZONE = 0.3;
    private static final int ELEVATOR_MOTOR_ID = 62; //CORRECT ID
    private static final int ELEVATOR_BOTTOM_SETPOINT = 0;
    private static final int ELEVATOR_MID_SETPOINT = TICKS_PER_INCH * 5; 
    private static final int ELEVATOR_TOP_SETPOINT = TICKS_PER_INCH * 28; 
    
    //self-initializes the class
    private static final Elevator instance = new Elevator();
    
    //PID constants
    private static final double P = 1.0;
    private static final double I = 0.00001;
    private static final double D = 0.01;
    
    //Motor and controller
    private final TalonFX elevatorMotor;
    private final DigitalInput bottomLimitSwitch;

    //variables
    private double encoderPosition;
    private double overShoot;
    private int targetSetpoint;
    private boolean isCalibrating;
    private ElevatorMode controlMode;
    private ElevatorSetpoint setpoint;
    
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

        elevatorMotor.configPeakOutputForward(0.7);
        elevatorMotor.configPeakOutputReverse(-0.7);
        
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
        instance.isCalibrating = false;
    }
    
    /**
     * The method that will be run in teleopPeriodic
     * @param speed - the speed of the elevator in manual mode
     * @param switchControlMode - toggles between manual and position control
     * @param resetEncoderPosition - sets the encoder position to zero
     * @param setPositionBottom - Sets the setpoint to the zero position 
     * @param setPositionMid - Sets the setpoint to the mid position
     * @param runAutoCalibrate - If pressed, robot will automatically determine zero point
     * @param setPositionTop - Sets the setpoint to the top position
     * @param dPadDirection - The direction of the dpad for low sensitivity control
     */
    public static void run(double speed, boolean switchControlMode, boolean resetEncoderPosition, boolean setPositionBottom, boolean setPositionMid, boolean runAutoCalibrate, boolean setPositionTop, int dPadDirection) {
        instance.encoderPosition = instance.elevatorMotor.getSensorCollection().getIntegratedSensorPosition();

        //runs auto calibrate or sets current encoder position to 0 if start button is pressed
        setEncoderZero(resetEncoderPosition);
        
        //switches between manual and position control modes
        if (switchControlMode) {
            if (instance.controlMode == ElevatorMode.POSITION) {
                instance.controlMode = ElevatorMode.MANUAL;
            } else {
                instance.controlMode = ElevatorMode.POSITION;
            }
        }

        //switches between setpoints
        updateTargetSetpoint(setPositionBottom, setPositionMid, setPositionTop);
        
        //runs control mode
        if (instance.controlMode == ElevatorMode.MANUAL) { //manual control
            manualControl(speed, dPadDirection);
        } if (instance.controlMode == ElevatorMode.POSITION) { //position control
            positionControl();
        }

        autoCalibrate(runAutoCalibrate);

        //prints variables to Smart Dashboard
        updateSmartDashboard(speed, dPadDirection);
    }

    /**
     * Manual control of the elevator using the left joystick
     */
    private static void manualControl(double speed, int dPadDirection) {

        if (Math.abs(speed) < MANUAL_DEADZONE || (!instance.bottomLimitSwitch.get() && speed > MANUAL_DEADZONE) || (instance.encoderPosition < -280000 && speed < -MANUAL_DEADZONE)) {
            speed = 0;
        } 
        
        //low sensitivity control using DPAD
        if (dPadDirection == 180) {
            speed = 0.2;
        } else if (dPadDirection == 0) {
            speed = -0.2;
        }

        instance.elevatorMotor.set(ControlMode.PercentOutput, speed * 0.6);

        //overwrite calibration if the elevator is moving
        if (speed != 0) {
            instance.isCalibrating = false;
        }
    }

    /**
     * PID Control of the elevator, cycles through
     * setpoints bottom, mid, top
     */
    private static void positionControl() {
        //calculates the overshoot in encoder ticks, used for tuning PID
        if (Math.abs(instance.encoderPosition) - Math.abs(instance.targetSetpoint)  > instance.overShoot && Math.abs(instance.encoderPosition) > Math.abs(instance.targetSetpoint)) {
            instance.overShoot = Math.abs(instance.encoderPosition) - Math.abs(instance.targetSetpoint);
        }

        //set elevator PID position to target setpoint
        instance.elevatorMotor.set(ControlMode.Position, instance.targetSetpoint);

        //overwrite calibration if the elevator is moving
        instance.isCalibrating = false;
    }

    /**
     * Sets the encoder position to 0 if the start button is pressed
     */
    private static void setEncoderZero(boolean resetEncoderPosition) {
        if (resetEncoderPosition || !instance.bottomLimitSwitch.get()) {
            instance.elevatorMotor.getSensorCollection().setIntegratedSensorPosition(0, 30);
        }
    }

    private static void updateTargetSetpoint(boolean setPositionBottom, boolean setPositionMid, boolean setPositionTop) {
        //updates the setpoint enum
        if (setPositionBottom) {
            instance.setpoint = ElevatorSetpoint.BOTTOM;
            instance.overShoot = 0;
        } else if (setPositionMid) {
            instance.setpoint = ElevatorSetpoint.MID;
            instance.overShoot = 0;
        } else if (setPositionTop) {
            instance.setpoint = ElevatorSetpoint.TOP;
            instance.overShoot = 0;
        }

        //updates targetSetpoint variable
        if (instance.setpoint == ElevatorSetpoint.BOTTOM) {
            instance.targetSetpoint = ELEVATOR_BOTTOM_SETPOINT;
        } else if (instance.setpoint == ElevatorSetpoint.MID) {
            instance.targetSetpoint = ELEVATOR_MID_SETPOINT;
        } else if (instance.setpoint == ElevatorSetpoint.TOP) {
            instance.targetSetpoint = ELEVATOR_TOP_SETPOINT;
        }
    }

    /**
     * Runs motor until limit switch is triggered. Interrupted by entering PID mode, pressing x again, or controlling motor in manual mode
     * @param runAutoCalibrate
     */
    private static void autoCalibrate(boolean runAutoCalibrate) {
        //toggle between calibrating and not calibrating
        if (runAutoCalibrate) {
            if (instance.isCalibrating) {
                instance.isCalibrating = false;
            } else {
                instance.isCalibrating = true;
            }
        }

        //calibrates encoder position
        if (instance.isCalibrating && instance.bottomLimitSwitch.get()) {
            instance.elevatorMotor.set(ControlMode.PercentOutput, 0.5);
        } else if (instance.isCalibrating && !instance.bottomLimitSwitch.get()) {
            instance.elevatorMotor.set(ControlMode.PercentOutput, 0);
            instance.elevatorMotor.getSensorCollection().setIntegratedSensorPosition(0, 30);

            instance.isCalibrating = false;
        }
    }

    private static void updateSmartDashboard(double speed, int dPadDirection) {
        SmartDashboard.putString("Elev Control Mode", instance.controlMode.toString());
        SmartDashboard.putString("Elev Setpoint", instance.setpoint.toString());
        SmartDashboard.putNumber("Elev Speed", speed);
        SmartDashboard.putNumber("Elev Position", instance.encoderPosition/TICKS_PER_INCH);
        SmartDashboard.putNumber("Elev Target Position", instance.targetSetpoint/TICKS_PER_INCH);
        SmartDashboard.putNumber("Elev Overshoot", instance.overShoot);
        SmartDashboard.putNumber("Elev Motor Power", instance.elevatorMotor.getMotorOutputPercent());
        SmartDashboard.putBoolean("Limit Switch", !instance.bottomLimitSwitch.get());
        SmartDashboard.putBoolean("Auto Calibrate", instance.isCalibrating);
        SmartDashboard.putNumber("POV", dPadDirection);
    }
}
