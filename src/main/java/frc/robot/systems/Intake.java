package frc.robot.systems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.subsystems.*;

public class Intake {
    //ID constants
    private static final int ROLLER_ID = RobotMap.INTAKE_MOTOR_ID;
    private static final int CLAW_FORWARD_CHANNEL = 8;
    private static final int CLAW_REVERSE_CHANNEL = 9;
    private static final int PIVOT_FORWARD_CHANNEL = 10;
    private static final int PIVOT_REVERSE_CHANNEL = 11;

    //The deadzone for roller joystick control
    private static final double ROLLER_DEADZONE = 0.3;

    private double rackMotorSpeed;

    private Compressor comp;
    private static Pivot pivot;
    private static Roller roller;
    private static Claw claw;

    private SolenoidState clawState;
    private SolenoidState pivotState;

    private static Intake instance = new Intake();

    public Intake() {
        comp = new Compressor(1, PneumaticsModuleType.CTREPCM);
        comp.enableDigital();

        //initiali
        roller = new Roller(ROLLER_ID);
        claw = new Claw(PneumaticsModuleType.CTREPCM, CLAW_FORWARD_CHANNEL, CLAW_REVERSE_CHANNEL);
        pivot = new Pivot(PIVOT_FORWARD_CHANNEL, PIVOT_REVERSE_CHANNEL);
    }

    private enum SolenoidState {
        OPEN, CLOSED, UP, DOWN
    }

    public static void init() {
        instance.rackMotorSpeed = 0;
        instance.pivotState = SolenoidState.UP;
        instance.clawState = SolenoidState.OPEN;
    }

    /**
     * The method to be run from teleopPeriodic
     * @param dPadDirection - the direction of the DPAD
     * @param rollerSpeed - The speed of the roller
     * @param clawButton - The button to extend/retract the claw
     * @param pivotButton - The button to extend/retract the pivot
     */
    public static void run(int dPadDirectionOne, int dPadDirectionTwo, double rightTriggerOne, double leftTriggerOne, double rightTriggerTwo, double leftTriggerTwo, boolean clawToggle) {
        if(Math.max(rightTriggerOne, rightTriggerTwo) > ROLLER_DEADZONE ) {
            runRoller(Math.max(rightTriggerOne, rightTriggerTwo));
        } else if(Math.max(leftTriggerOne, leftTriggerTwo) > ROLLER_DEADZONE) {
            runRoller(-Math.max(leftTriggerOne, leftTriggerTwo));
        } else {
            runRoller(0);
        }

        runClaw(clawToggle);
        runPivot(dPadDirectionOne, dPadDirectionTwo);

        updateSmartDashboard();
    }

    /**
     * Runs the rollers, operated with a joystick axis
     * @param rollerSpeed the speed of the rollers from -1 to 1
     */
    private static void runRoller(double rollerSpeed) {
        if(Math.abs(rollerSpeed) > ROLLER_DEADZONE) { // scales down the speed of the motor
            rollerSpeed *= 0.8;
        } else {
            rollerSpeed = 0;
        }

        roller.setSpeed(rollerSpeed);
    }

    /**
     * Extends or retracts the claw, toggled with button press
     * @param switchClawState
     */
    private static void runClaw(boolean switchClawState) {
        if(switchClawState) {
            if(instance.clawState == SolenoidState.OPEN) {
                instance.clawState = SolenoidState.CLOSED;
        } else {
                instance.clawState = SolenoidState.OPEN;
            }
        } 

        if(instance.clawState == SolenoidState.OPEN) {
            claw.open();
        } else {
            claw.close();
        }
    }  
    
    /**
     * Extends or retracts the pivot, toggled with button press
     * @param switchPivotState
     */
    private static void runPivot(int dPadDirectionOne, int dPadDirectionTwo) {

        //Cancels out switching pivot if the two dpads are facing opposite directions
        if(Math.abs(dPadDirectionOne - dPadDirectionTwo) == 180) {
            return;
        } 

        if(dPadDirectionOne == 0 || dPadDirectionTwo == 0) {
            instance.pivotState = SolenoidState.UP;
        } else if(dPadDirectionOne == 180 || dPadDirectionTwo == 180) {
            instance.pivotState = SolenoidState.DOWN;
        }

        if(instance.pivotState == SolenoidState.DOWN) {
            pivot.down();
        } else {
            pivot.up();
        }
    }

    /**
     * Updates Smart Dashboard with important variables
     * @param rollerSpeed
     */
    private static void updateSmartDashboard() {
        SmartDashboard.putString("Claw State", instance.clawState.toString());
        SmartDashboard.putString("Pivot State", instance.pivotState.toString());
        SmartDashboard.putNumber("Rack Speed", instance.rackMotorSpeed);
    }
    
}
