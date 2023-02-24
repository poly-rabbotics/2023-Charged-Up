package frc.robot.systems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.*;

public class Intake {
    //ID constants
    private static final int RACK_MOTOR_ID = 1;
    private static final int ROLLER_ID = 0;
    private static final int CLAW_FORWARD_CHANNEL = 2;
    private static final int CLAW_REVERSE_CHANNEL = 3;
    private static final int PIVOT_FORWARD_CHANNEL = 4;
    private static final int PIVOT_REVERSE_CHANNEL = 5;

    //The deadzone for roller joystick control
    private static final double ROLLER_DEADZONE = 0.3;

    private double rackMotorSpeed;

    private Compressor comp;
    private static Rack rack;
    private static Pivot pivot;
    private static Roller roller;
    private static Claw claw;

    private SolenoidState clawState;
    private SolenoidState pivotState;

    private static Intake instance = new Intake();

    public Intake() {
        comp = new Compressor(1, PneumaticsModuleType.CTREPCM);
        comp.enableDigital();

        //UNCOMMENT LATER
        roller = new Roller(ROLLER_ID);
        claw = new Claw(PneumaticsModuleType.CTREPCM, CLAW_FORWARD_CHANNEL, CLAW_REVERSE_CHANNEL);
        pivot = new Pivot(PIVOT_FORWARD_CHANNEL, PIVOT_REVERSE_CHANNEL);
        rack = new Rack(RACK_MOTOR_ID);
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
    public static void run(int dPadDirection, double rollerSpeed, boolean clawButton, boolean pivotButton) {
        runRack(dPadDirection);
        runRoller(rollerSpeed);
        runClaw(clawButton);
        runPivot(pivotButton);

        updateSmartDashboard(rollerSpeed);
    }

    /**
     * Runs the rack motor, operated with DPAD left/right
     * @param dPadDirection the direction of the DPAD
     */
    private static void runRack(int dPadDirection) {
        if(dPadDirection == 90) {
            instance.rackMotorSpeed = 0.8;
        } else if(dPadDirection == 270) {
            instance.rackMotorSpeed = -0.8;
        } else {
            instance.rackMotorSpeed = 0;
        }

        rack.setSpeed(instance.rackMotorSpeed);
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
        } else {
            pivot.up();
        }
    }

    /**
     * Updates Smart Dashboard with important variables
     * @param rollerSpeed
     */
    private static void updateSmartDashboard(double rollerSpeed) {
        SmartDashboard.putString("Claw State", instance.clawState.toString());
        SmartDashboard.putString("Pivot State", instance.pivotState.toString());
        SmartDashboard.putNumber("Rack Speed", instance.rackMotorSpeed);
        SmartDashboard.putNumber("Roller Speed", rollerSpeed);
    }
    
}
