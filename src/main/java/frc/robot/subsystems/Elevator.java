package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Elevator {
    
    private static final int ELEVATOR_MOTOR_ID = 0;
    private static final int ELEVATOR_BOTTOM_SETPOINT = 0;
    private static final int ELEVATOR_TOP_SETPOINT = 10000; //find the real value, this is arbitrary for now
    
    private double speed;
    private double position;
    private boolean aPressed = false;
    private String controlMode;
    
    TalonFX elevatorMotor;
    XboxController controller; //get rid of this once merged, we need to use a universal controller
    
    private static Elevator instance = new Elevator();
    
    private Elevator() {
        elevatorMotor = new TalonFX(ELEVATOR_MOTOR_ID);
        controller = new XboxController(0);
        
        instance.controlMode = "Manual";
    }
    
    /**
    * The main method for the elevator that will be run from other classes
    */
    public static void run() {
        instance.speed = instance.getElevatorSpeed();
        
        //switches between manual and position control modes
        if(instance.getSwitchControlMode()) {
            if(instance.controlMode.equals("Position")) {
                instance.controlMode = "Manual";
            } else {
                instance.controlMode = "Position";
            }
        }
        
        //sets the elevator motor to percent output if in manual mode
        if(instance.controlMode.equals("Manual")) {
            if(instance.speed >= 0.1 || instance.speed <= -0.1) {
                instance.elevatorMotor.set(ControlMode.PercentOutput, instance.speed);
            } else {
                instance.elevatorMotor.set(ControlMode.PercentOutput, 0);
            }
        } 
        
        if(instance.controlMode.equals("Position")) {
            instance.elevatorMotor.set(ControlMode.Position, instance.position);
        }
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
        if(!aPressed && controller.getRawButton(1)) {
            aPressed = true;
            return true;
        } else if(aPressed && !controller.getRawButton(1)) {
            aPressed = false;
            return false;
        } else return false;
    }
}
