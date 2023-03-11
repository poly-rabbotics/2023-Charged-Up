package frc.robot.systems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;

public class Intake {
    //ID constants
    private static final int ROLLER_ID = 0;
    private static final int CLAW_FORWARD_CHANNEL = 1;
    private static final int CLAW_REVERSE_CHANNEL = 0;
    private static final int PIVOT_FORWARD_CHANNEL = 2;
    private static final int PIVOT_REVERSE_CHANNEL = 3;

    //The deadzone for roller joystick control
    private static final double ROLLER_DEADZONE = 0.3;

    private double rackMotorSpeed;

    public static Compressor comp;
    private static Pivot pivot;
    private static Roller roller;
    private static Claw claw;

    private SolenoidState clawState;
    private SolenoidState pivotState;

    private static Intake instance = new Intake();

    public Intake() {
        comp = new Compressor(1, PneumaticsModuleType.REVPH);
        //comp.enableDigital();

        //initiali
        roller = new Roller(ROLLER_ID);
        claw = new Claw(PneumaticsModuleType.REVPH, CLAW_FORWARD_CHANNEL, CLAW_REVERSE_CHANNEL);
        pivot = new Pivot(PIVOT_FORWARD_CHANNEL, PIVOT_REVERSE_CHANNEL);
    }

    public enum SolenoidState {
        OPEN, CLOSED, UP, DOWN
    }

    public static void init() {
        instance.rackMotorSpeed = 0;  
        instance.pivotState = SolenoidState.DOWN;
        instance.clawState = SolenoidState.CLOSED;
    }

    /**
     * The method to be run from teleopPeriodic
     * @param dPadDirection - the direction of the DPAD
     * @param rollerSpeed - The speed of the roller
     * @param clawButton - The button to extend/retract the claw
     * @param pivotButton - The button to extend/retract the pivot
     */
    public static void run(boolean pivotToggle, boolean intake, boolean outtake, boolean clawToggle) {
        if(intake) {
            runRoller(1);
        } else if(outtake) {
            runRoller(-1);
        } else {
            runRoller(0);
        }

        runClaw(clawToggle);
        runPivot(pivotToggle);

        updateSmartDashboard();
    }

    public static void autoClaw(SolenoidState state) {
        if(state == SolenoidState.OPEN) {
            claw.open();
        } else if(state == SolenoidState.CLOSED) {
            claw.close();
        }
    }

    public static void autoPivot(SolenoidState state) {
        if(state == SolenoidState.DOWN) {
            pivot.down();
        } else if(state == SolenoidState.UP) {
            pivot.up();
        }
    }

    /**
     * Runs the rollers, operated with a joystick axis
     * @param rollerSpeed the speed of the rollers from -1 to 1
     */
    private static void runRoller(double rollerSpeed) {
        if(Math.abs(rollerSpeed) > ROLLER_DEADZONE) { // scales down the speed of the motor
            rollerSpeed *= 0.5;
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
        } else if(instance.clawState == SolenoidState.CLOSED) {
            claw.close();
        }
    }  
    
    /**
     * Extends or retracts the pivot, toggled with button press
     * @param switchPivotState
     */
    private static void runPivot(boolean switchPivotState) {
        if(switchPivotState) {
            if(instance.pivotState == SolenoidState.UP) {
                instance.pivotState = SolenoidState.DOWN;
            } else {
                instance.pivotState = SolenoidState.UP;
            }
        }

        if(instance.pivotState == SolenoidState.DOWN) {
            pivot.down();
        } else if(instance.pivotState == SolenoidState.UP) {
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
