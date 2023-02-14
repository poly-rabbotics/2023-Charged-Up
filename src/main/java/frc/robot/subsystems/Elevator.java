package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator {

    /*
     * *******************
     * 1 INCH = 18500 TICKS (probably accurate)
     * *******************
     */
    private static final int TICKS_PER_INCH = -4000; //temp value
    
    private static final int ELEVATOR_MOTOR_ID = 5;
    private static final int ELEVATOR_BOTTOM_SETPOINT = 0;
    private static final int ELEVATOR_MID_SETPOINT = -500000; //find the real value, this is arbitrary for now
    private static final int ELEVATOR_TOP_SETPOINT = -250000; //find the real value, this is arbitrary for now
    private static final double MANUAL_DEADZONE = 0.3;

    private double speed;
    private double encoderPosition;
    private boolean rbPressed = false;
    private ElevatorMode controlMode;
    private ElevatorSetpoint setpoint;

    private double P = 0.000000001;
    private double I = 0.0;
    private double D = 0.0;
    
    TalonFX elevatorMotor;
    XboxController controller; //get rid of this once merged, we need to use a universal controller
    
    private static Elevator instance = new Elevator();
    
    private Elevator() {
        elevatorMotor = new TalonFX(ELEVATOR_MOTOR_ID);
        controller = new XboxController(0);

        elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        elevatorMotor.config_kP(0, P);
        elevatorMotor.config_kI(0, I);
        elevatorMotor.config_kD(0, D);
    }

    private enum ElevatorMode {
        MANUAL, POSITION
    }

    private enum ElevatorSetpoint {
        BOTTOM, MID, TOP
    }

    public static void elevatorInit() {
        instance.controlMode = ElevatorMode.MANUAL;
        instance.setpoint = ElevatorSetpoint.BOTTOM;
    }
    
    /**
    * The main method for the elevator that will be run from other classes
    */
    public static void run() {
        instance.speed = getElevatorSpeed();
        instance.speed = instance.elevatorMotor.getSelectedSensorPosition();

        

        //sets current encoder position to 0 if x button is pressed
        if(instance.controller.getRawButton(3)) {
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
        
        //sets the elevator motor to percent output if in manual mode
        if(instance.controlMode == ElevatorMode.MANUAL) {
            manualControl();
        } 
        
        if(instance.controlMode == ElevatorMode.POSITION) {
            positionControl();
        }

        SmartDashboard.putBoolean("Manual Mode", instance.controlMode == ElevatorMode.MANUAL);
        SmartDashboard.putBoolean("Setpoint", instance.setpoint == ElevatorSetpoint.BOTTOM);
        SmartDashboard.putNumber("Speed", getElevatorSpeed());
        SmartDashboard.putNumber("Encoder position", instance.elevatorMotor.getSelectedSensorPosition());
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

        //sets setpoint to bottom if a button is pressed
        if(instance.controller.getRawButton(1))
            instance.setpoint = ElevatorSetpoint.BOTTOM;
        //sets setpoint to mid if b button is pressed
        else if(instance.controller.getRawButton(2))
            instance.setpoint = ElevatorSetpoint.MID;
        //sets setpoint to top if y button is pressed
        else if(instance.controller.getRawButton(4))
            instance.setpoint = ElevatorSetpoint.TOP;


        if(instance.setpoint == ElevatorSetpoint.BOTTOM) {
            instance.elevatorMotor.set(ControlMode.Position, ELEVATOR_BOTTOM_SETPOINT);
        } else if(instance.setpoint == ElevatorSetpoint.MID) {
            instance.elevatorMotor.set(ControlMode.Position, ELEVATOR_MID_SETPOINT);
        } else if(instance.setpoint == ElevatorSetpoint.TOP) {
            instance.elevatorMotor.set(ControlMode.Position, ELEVATOR_TOP_SETPOINT);
        }
    }
}
