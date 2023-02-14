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
    private static final int TICKS_PER_INCH = 18500;
    
    private static final int ELEVATOR_MOTOR_ID = 0;
    private static final int ELEVATOR_BOTTOM_SETPOINT = 0;
    private static final int ELEVATOR_MID_SETPOINT = 15 * TICKS_PER_INCH; //find the real value, this is arbitrary for now
    private static final int ELEVATOR_TOP_SETPOINT = 30 * TICKS_PER_INCH; //find the real value, this is arbitrary for now
    private static final double MANUAL_DEADZONE = 0.1;

    private double speed;
    private double encoderPosition;
    private boolean rbPressed = false;
    private ElevatorMode controlMode;
    private ElevatorSetpoint setpoint;

    private double P = 0.0000001;
    private double I = 0.0;
    private double D = 0.0;
    
    TalonFX elevatorMotor;
    XboxController controller; //get rid of this once merged, we need to use a universal controller
    
    private static Elevator instance = new Elevator();
    
    private Elevator() {
        elevatorMotor = new TalonFX(ELEVATOR_MOTOR_ID);
        controller = new XboxController(0);
        
        controlMode = ElevatorMode.POSITION;
        setpoint = ElevatorSetpoint.BOTTOM;

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
    
    /**
    * The main method for the elevator that will be run from other classes
    */
    public static void run() {
        instance.speed = instance.getElevatorSpeed();
        instance.speed = instance.elevatorMotor.getSelectedSensorPosition();
        
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

        SmartDashboard.putBooleanArray("Setpoint", new boolean[] {instance.setpoint == ElevatorSetpoint.BOTTOM, instance.setpoint == ElevatorSetpoint.MID, instance.setpoint == ElevatorSetpoint.TOP});
    }
    
    /**
    * Gets the left joystick's value to be used for percent output
    * @return Left joystick Y axis
    */
    private double getElevatorSpeed() {
        return -(controller.getRawAxis(1));
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
        if(instance.speed >= MANUAL_DEADZONE || instance.speed <= -MANUAL_DEADZONE) {
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

        //sets current encoder position to 0 if x button is pressed
        if(instance.controller.getRawButton(3))
            instance.elevatorMotor.setSelectedSensorPosition(0);


        if(instance.setpoint == ElevatorSetpoint.BOTTOM) {
            instance.elevatorMotor.set(ControlMode.Position, ELEVATOR_BOTTOM_SETPOINT);
        } else if(instance.setpoint == ElevatorSetpoint.MID) {
            instance.elevatorMotor.set(ControlMode.Position, ELEVATOR_MID_SETPOINT);
        } else if(instance.setpoint == ElevatorSetpoint.TOP) {
            instance.elevatorMotor.set(ControlMode.Position, ELEVATOR_TOP_SETPOINT);
        }
    }
}
