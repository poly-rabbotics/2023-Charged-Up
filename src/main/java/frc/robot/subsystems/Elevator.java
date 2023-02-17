package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator {

    private static final int TICKS_PER_INCH = -23890; //temp value
    
    private static final int ELEVATOR_MOTOR_ID = 5;
    private static final int ELEVATOR_BOTTOM_SETPOINT = 0;
    private static final int ELEVATOR_MID_SETPOINT = TICKS_PER_INCH * 16; //find the real value, this is arbitrary for now
    private static final int ELEVATOR_TOP_SETPOINT = TICKS_PER_INCH * 28; //find the real value, this is arbitrary for now
    private static final double MANUAL_DEADZONE = 0.3;

    private int targetSetpoint;
    private double speed;
    private double encoderPosition;
    private boolean rbPressed = false;
    private ElevatorMode controlMode;
    private ElevatorSetpoint setpoint;
    private boolean recordOvershoot = false;
    private double overShoot;

    private final double P = 1.0;
    private final double I = 0.00001;
    private final double D = 0.01;
    
    TalonFX elevatorMotor;
    XboxController controller; //get rid of this once merged, we need to use a universal controller
    
    private static Elevator instance = new Elevator();
    
    private Elevator() {
        elevatorMotor = new TalonFX(ELEVATOR_MOTOR_ID);
        controller = new XboxController(0);

        elevatorMotor.configFactoryDefault();
        elevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
        elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
        elevatorMotor.config_kP(0, P);
        elevatorMotor.config_kI(0, I);
        elevatorMotor.config_kD(0, D);
        elevatorMotor.selectProfileSlot(0, 0);
        
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
    public static void elevatorInit() {
        instance.controlMode = ElevatorMode.MANUAL;
        instance.setpoint = ElevatorSetpoint.BOTTOM;
        instance.elevatorMotor.setNeutralMode(NeutralMode.Brake);
    }
    
    /**
    * The main method for the elevator that will be run from other classes
    */
    public static void run() {
        instance.speed = getElevatorSpeed();
        instance.encoderPosition = instance.elevatorMotor.getSensorCollection().getIntegratedSensorPosition();

        //sets current encoder position to 0 if start button is pressed
        if(instance.controller.getRawButton(8)) {
            instance.elevatorMotor.getSensorCollection().setIntegratedSensorPosition(0, 30);
        }
        
        //switches between manual and position control modes
        if(instance.getSwitchControlMode()) {
            if(instance.controlMode == ElevatorMode.POSITION) {
                instance.controlMode = ElevatorMode.MANUAL;
            } else {
                instance.controlMode = ElevatorMode.POSITION;
            }
        }
        
        if(instance.controlMode == ElevatorMode.MANUAL) { //manual control
            manualControl();
        } if(instance.controlMode == ElevatorMode.POSITION) { //position control
            positionControl();
        }

        //updates the smart dashboard
        SmartDashboard.putString("Elev Control Mode", instance.controlMode.toString());
        SmartDashboard.putString("Elev Setpoint", instance.setpoint.toString());
        SmartDashboard.putNumber("Elev Speed", getElevatorSpeed());
        SmartDashboard.putNumber("Elev Position", instance.encoderPosition);
        SmartDashboard.putNumber("Elev Target Position", instance.targetSetpoint);
        SmartDashboard.putNumber("Elev Overshoot", instance.overShoot);
        SmartDashboard.putNumber("Elev Motor Power", instance.elevatorMotor.getMotorOutputPercent());
    }
    
    /**
    * Gets the left joystick's value to be used for percent output
    * @return Left joystick Y axis
    */
    private static double getElevatorSpeed() {
        return (instance.controller.getRawAxis(1));
    }
    
    /**
    * Gets the value of the A button to be used for switching control modes, 
    * only returns true once until relesed and pressed again.
    */
    private boolean getSwitchControlMode() {
        if(!rbPressed && controller.getRawButton(6)) {
            rbPressed = true;
            return true;
        } else if(rbPressed && !controller.getRawButton(6)) {
            rbPressed = false;
            return false;
        } else return false;
    }

    /**
     * Manual control of the elevator using the left joystick
     */
    private static void manualControl() {
        instance.speed = getElevatorSpeed();

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

        if(instance.controller.getRawButton(1)) {//A button
            instance.setpoint = ElevatorSetpoint.BOTTOM;
            instance.overShoot = 0;
            instance.recordOvershoot = false;
        } else if(instance.controller.getRawButton(2)) {//B button
            instance.setpoint = ElevatorSetpoint.MID;
            instance.overShoot = 0;
            instance.recordOvershoot = false;
        } else if(instance.controller.getRawButton(4)) {//Y button
            instance.setpoint = ElevatorSetpoint.TOP;
            instance.overShoot = 0;
        }

        //sets elevator motor to current setpoint
        if(instance.setpoint == ElevatorSetpoint.BOTTOM) {
            instance.targetSetpoint = ELEVATOR_BOTTOM_SETPOINT;
        } else if(instance.setpoint == ElevatorSetpoint.MID) {
            instance.targetSetpoint = ELEVATOR_MID_SETPOINT;
        } else if(instance.setpoint == ElevatorSetpoint.TOP) {
            instance.targetSetpoint = ELEVATOR_TOP_SETPOINT;
        }

        if(Math.abs(instance.encoderPosition) - Math.abs(instance.targetSetpoint)  > instance.overShoot && Math.abs(instance.encoderPosition) > Math.abs(instance.targetSetpoint)) {
            instance.overShoot = Math.abs(instance.encoderPosition) - Math.abs(instance.targetSetpoint);
        }

        instance.elevatorMotor.set(ControlMode.Position, instance.targetSetpoint);
    }
}
