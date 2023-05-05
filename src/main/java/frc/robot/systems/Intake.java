package frc.robot.systems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.systems.ElevFourbar.GamePiece;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.SmartPrintable;
import frc.robot.subsystems.*;

public class Intake extends SmartPrintable {

    //Enum for claw and pivot states
    public enum SolenoidState {
        OPEN, CLOSED, UP, DOWN
    }

    //ID constants
    private static final int ROLLER_ID = 0;
    private static final int CLAW_FORWARD_CHANNEL = 1;
    private static final int CLAW_REVERSE_CHANNEL = 0;
    private static final int PIVOT_FORWARD_CHANNEL = 2;
    private static final int PIVOT_REVERSE_CHANNEL = 3;

    //Declare objects
    public static Compressor comp;
    private static Pivot pivot;
    private static Roller roller;
    private static Claw claw;

    private Timer timer = new Timer();

    //instantiate instance of class
    private static Intake instance = new Intake();
    

    public Intake() {
        super();

        comp = new Compressor(0, PneumaticsModuleType.CTREPCM);
        comp.enableDigital();

        //initialize objects
        roller = new Roller(ROLLER_ID);
        claw = new Claw(PneumaticsModuleType.CTREPCM, CLAW_FORWARD_CHANNEL, CLAW_REVERSE_CHANNEL);
        pivot = new Pivot(PIVOT_FORWARD_CHANNEL, PIVOT_REVERSE_CHANNEL);
    }

    //Opens claw if cube is selected, closes claw if cone is selected
    public static void init() {
        claw.close();

        instance.timer.reset();
        instance.timer.start();
    }

    /**
     * Runs the intake
     * @param pivotToggle Toggles the pivot's state when pressed
     * @param clawHeld Opens the claw when held
     * @param clawReleased 
     * @param intake
     * @param outtake
     */
    public static void run(boolean pivotToggle, boolean clawHeld, boolean clawReleased, boolean intake, boolean outtake) {
        double rollerSpeed;

        //IF CUBE IS SELECTED
        if(ElevFourbar.getGamePieceSelected() == GamePiece.CUBE) {
            if(intake) { //Run rollers in at full speed and open claw
                rollerSpeed = -1.0;
                clawHeld = true;
            }
            else if(clawHeld) //Dispense cubes at low power
                rollerSpeed = 0.3;

            else if(outtake) //Shoot cubes at full power
                rollerSpeed = 1.0;

            else //Idly run at very low power to keep in game pieces
                rollerSpeed = -0.06;
        } 

        //IF CONE IS SELECTED
        else {
            if(intake) //Intake at full speed
                rollerSpeed = -1.0;
            
            else if(outtake) //Outtake at full speed
                rollerSpeed = 1.0;

            else //Idly run at very low power to keep in game pieces
                rollerSpeed = -0.06;
        }
        
        //Run the mechanisms
        runClaw(clawHeld, clawReleased);
        runPivot(pivotToggle);
        runRoller(rollerSpeed);
    }

    /**
     * Runs the claw
     * @param held Opens claw if button is held
     * @param released Restarts auto intake timer
     */
    private static void runClaw(boolean held, boolean released) {
        if (held) {
            claw.open();
        } else {
            claw.close();
        }
    }

    /**
     * @param toggle toggle between up and down when true
     */
    private static void runPivot(boolean toggle) {
        if(toggle) {
            if(pivot.getState() == SolenoidState.UP) {
                pivot.down();
            } else {
                pivot.up();
            }
        }
    }

    /**
     * @param speed speed of the roller
     */
    public static void runRoller(double speed) {
        roller.setSpeed(speed);
    }

    /**
     * @param state state to set claw to
     */
    public static void autoClaw(SolenoidState state) {
        if(state == SolenoidState.OPEN) {
            claw.open();
        } else if(state == SolenoidState.CLOSED) {
            claw.close();
        }
    }

    /**
     * @param state state to set pivot to
     */
    public static void autoPivot(SolenoidState state) {
        if(state == SolenoidState.DOWN) {
            pivot.down();
        } else if(state == SolenoidState.UP) {
            pivot.up();
        }
    }

    /**
     * Run the roller for a set amount of time
     * @param startTime 
     * @param duration
     * @param speed
     */
    public static void autoRoller(double startTime, double duration, double speed) {
        if(instance.timer.get() > startTime && instance.timer.get() < startTime + duration) {
            roller.setSpeed(speed);
        } else {
            roller.setSpeed(0);
        }
    }

    /**
     * @return current state of the pivot
     */
    public static SolenoidState getPivotState() {
        return pivot.getState();
    }

    /**
     * @return current state of the claw
     */
    public static SolenoidState getClawState() {
        return claw.getState();
    }


    /**
     * Prints needed information to the SmartDashboard through the SmartPrintable class
     */
    @Override
    public void print() {
        SmartDashboard.putString("Claw State", claw.getState().toString());
        SmartDashboard.putString("Pivot State", pivot.getState().toString());
    }
}
