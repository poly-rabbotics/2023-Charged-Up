package frc.robot.systems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SmartPrintable;
import frc.robot.subsystems.*;
import frc.robot.systems.ElevFourbar.GamePiece;

public class Intake extends SmartPrintable {

    public enum SolenoidState {
        OPEN, CLOSED, UP, DOWN
    }

    //ID constants
    private static final int ROLLER_ID = 0;
    private static final int CLAW_FORWARD_CHANNEL = 1;
    private static final int CLAW_REVERSE_CHANNEL = 0;
    private static final int PIVOT_FORWARD_CHANNEL = 2;
    private static final int PIVOT_REVERSE_CHANNEL = 3;
    
    private static double rollerStartTime;

    //declare objects
    public static Compressor comp;
    private static Pivot pivot;
    private static Roller roller;
    private static Claw claw;
    private static Timer timer = new Timer();

    private SolenoidState clawState;
    private SolenoidState pivotState;

    //instantiate instance of class
    private static Intake instance = new Intake();

    public Intake() {
        super();

        comp = new Compressor(1, PneumaticsModuleType.REVPH);
        //comp.enableDigital();

        //initiali
        roller = new Roller(ROLLER_ID);
        claw = new Claw(PneumaticsModuleType.REVPH, CLAW_FORWARD_CHANNEL, CLAW_REVERSE_CHANNEL);
        pivot = new Pivot(PIVOT_FORWARD_CHANNEL, PIVOT_REVERSE_CHANNEL);
        timer.start();
    }

    public static void init() { //opens claw if cube is selected, closes claw if cone is selected
        instance.pivotState = SolenoidState.DOWN;
        instance.clawState = (ElevFourbar.getGamePieceSelected() == ElevFourbar.GamePiece.CONE) ? SolenoidState.CLOSED : SolenoidState.OPEN;
        timer.reset();
        timer.start();
    }

    public static void autoInit() {
        timer.reset();
        timer.start();
    }

    /**
     * The method to be run from teleopPeriodic
     * @param dPadDirection - the direction of the DPAD
     * @param rollerSpeed - The speed of the roller
     * @param clawButton - The button to extend/retract the claw
     * @param pivotButton - The button to extend/retract the pivot
     */
    public static void run(boolean pivotToggle, boolean intake, boolean outtake, boolean clawHeld, boolean clawReleased) {
        instance.clawState = claw.getState();

        if(ElevFourbar.getGamePieceSelected() == GamePiece.CUBE) {
            //dispense cubes at low power
            if(intake) {
                runRoller(-1.0);
                clawHeld = true;
            }
            else if(clawHeld)
                runRoller(0.3);
            else if(outtake)
                runRoller(1.0);
            else 
                runRoller(0);

            runClaw(clawHeld, clawReleased);
        } else {
            runClaw(clawHeld, clawReleased);

            runRoller(intake, outtake);
        }

        runPivot(pivotToggle);
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
     * Automatically runs rollers for a set amount of time
     * @param startTime the time to start the rollers
     * @param endTime the time to stop the rollers
     * @param speed the speed of the rollers
     */
    public static void autoRoller(double startTime, double endTime, double speed) { 
        //if the timer is between the start and end times, run the rollers at the given speed
        if(timer.get() > startTime && timer.get() < endTime) {
            roller.setSpeed(speed);
        }
    }

    /**
     * Runs the roller at speeds determined by the state of the claw and the game piece selected
     * @param intake
     * @param outtake
     */
    public static void runRoller(boolean intake, boolean outtake) {
        double rollerSpeed;

        if(intake) {
            //intake at 100 percent
            rollerSpeed = -1.0;
        } else if(outtake) {
            //in cube mode, outtake at 100 percent if closed (shooting) and 60 percent if open (dispensing)
            if(ElevFourbar.getGamePieceSelected() == ElevFourbar.GamePiece.CUBE) {
                if(instance.clawState == SolenoidState.CLOSED) 
                    rollerSpeed = 1.0;
                else rollerSpeed = 0.6;
            } else {
                //outtake at 100 percent if in cone mode
                rollerSpeed = 1.0;
            }
        } else {
            //idly run rollers at 6 percent to secure game pieces
            rollerSpeed = -0.06;
        }

        roller.setSpeed(rollerSpeed);
    }

    /**
     * Runs the roller at a given speed
     * @param speed
     */
    public static void runRoller(double speed) {
        roller.setSpeed(speed);
    }

    /**
     * Extends or retracts the claw, toggled with button press
     * @param switchClawState
     */
    private static void runClaw(boolean switchClawState, boolean closeClaw) {
            if (switchClawState) {
                claw.open();
            }
            else {
                claw.close();
                autoRoller(rollerStartTime, rollerStartTime + 0.5, -1);
            }

            if (closeClaw) {
                claw.close();
                rollerStartTime = timer.get();
                
            }

            SmartDashboard.putNumber("Roller start time", rollerStartTime);
            SmartDashboard.putNumber("Rotime", timer.get());
            SmartDashboard.putBoolean("Close claw", switchClawState);
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

    public static SolenoidState getPivotState() {
        return instance.pivotState;
    }

    /**
     * Updates Smart Dashboard with important variables
     * @param rollerSpeed
     */
    public void print() {
        SmartDashboard.putString("Claw State", instance.clawState.toString());
        SmartDashboard.putString("Pivot State", instance.pivotState.toString());
    }
    
}
