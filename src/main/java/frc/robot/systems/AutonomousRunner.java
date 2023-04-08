package frc.robot.systems;

import java.security.InvalidParameterException;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.systems.ElevFourbar.Setpoint;
import frc.robot.systems.Intake.SolenoidState;
import frc.robot.SmartPrintable;

public class AutonomousRunner extends SmartPrintable {
    private static final double OUTTAKE_SPEED_CUBE = 0.3;
    private static final double OUTTAKE_SPEED_CONE = 0.4;

    private static final Runnable[] MODES = {
        () -> {},
        AutonomousRunner::modeOne,
        AutonomousRunner::modeTwo,
        AutonomousRunner::modeThree,
        AutonomousRunner::modeFour,
        AutonomousRunner::modeFive,
        AutonomousRunner::modeSix,
        AutonomousRunner::modeSeven,
    };

    private static final AutonomousRunner instance = new AutonomousRunner();

    private Timer timer;
    private Timer secondaryTimer;
    
    private int autoMode;
    private int autoStage;

    private AutonomousRunner() {
        super();

        timer = new Timer();
        secondaryTimer = new Timer();

        autoMode = 0;
        autoStage = 0;
    }

    /**
     * Set the current autonomous mode, should probably be called only once in
     * the autonomous initialization stage.
     */
    public static void setAutoMode(int mode) {
        if (mode < 0 || mode >= MODES.length) {
            throw new InvalidParameterException("Auto mode must be between 0 and 7 (inclusive)");
        }

        instance.autoMode = mode;
    }
    
    public static void init() {
        instance.timer.reset();
        instance.timer.start();
        
        instance.secondaryTimer.reset();
        instance.secondaryTimer.start();
        
        instance.autoStage = 0;
    }
    
    public static void run() {
        SmartDashboard.putNumber("Auto Stage", instance.autoStage);

        //Run the auto mode
        MODES[instance.autoMode].run();
    }

    /**
     * Scores mid then moves out of community
     */
    private static void modeOne() {

        score(Setpoint.MID_SCORING);
        
        /*******************
         * EXIT THE COMUNITY
         * *****************    
         */
        if (SwerveDrive.getDistance() < 670 && (instance.autoStage == 2 || instance.timer.get() > 10)) {
            SwerveDrive.runUncurved(0.0, -0.6, 0.0);
        } else {
            SwerveDrive.runUncurved(0.0, 0.0, 0.0);
        }
    }

    /**
     * Score only mid
     */
    private static void modeTwo() {
        score(Setpoint.MID_SCORING);
    }

    /**
     * Score mid then auto balance
     */
    private static void modeThree() {

        score(Setpoint.MID_SCORING);
        
        if (instance.autoStage > 1 || instance.timer.get() > 6) {
            AutoBalance.run();
        }
    }

    /**
     * Score high then move out of the community
     */
    private static void modeFour() {

        score(Setpoint.HIGH_SCORING);

        
        /*******************
         * EXIT THE COMUNITY
         * *****************    
         */
        if (SwerveDrive.getDistance() < 670 && (instance.autoStage == 2 || instance.timer.get() > 10)) {
            SwerveDrive.runUncurved(0.0, -0.6, 0.0);
        } else {
            SwerveDrive.runUncurved(0.0, 0.0, 0.0);
        }
    }

    /**
     * Only score high
     */
    private static void modeFive() {
        score(Setpoint.HIGH_SCORING);
    }

    /**
     * Score high and auto balance
     */
    private static void modeSix() {
        score(Setpoint.HIGH_SCORING);
        
        if (instance.autoStage > 1 || instance.timer.get() > 6) {
            AutoBalance.run();
        }
    }

    /**
     * Only drop game piece and move out of the community
     */
    private static void modeSeven() {
        AutoBalance.run();
    }

    private static void score(Setpoint setpoint) {
        if(instance.autoStage == 0){
            //Move the pivot up
            Intake.autoPivot(SolenoidState.UP);
            if(setpoint == Setpoint.HIGH_SCORING) 
                ElevFourbar.autoRun((ElevFourbar.gamePieceSelected == ElevFourbar.GamePiece.CUBE) ? ElevFourbar.HIGH_SCORING_COORDS_CUBE : ElevFourbar.HIGH_SCORING_COORDS_CONE);

            else if(setpoint == Setpoint.MID_SCORING)
                ElevFourbar.autoRun((ElevFourbar.gamePieceSelected == ElevFourbar.GamePiece.CUBE) ? ElevFourbar.MID_SCORING_COORDS_CUBE : ElevFourbar.MID_SCORING_COORDS_CONE);

            else
                ElevFourbar.autoRun(setpoint);
            
            //Move the elevator to the high scoring position
            if(instance.timer.get() > 2) {
                //Open claw when the position has been reached
                instance.secondaryTimer.reset();
                Intake.runRoller((ElevFourbar.gamePieceSelected == ElevFourbar.GamePiece.CUBE) ? OUTTAKE_SPEED_CUBE : OUTTAKE_SPEED_CONE); //run intake out
                instance.autoStage++;
            }
            if(instance.timer.get() > 2.1) Intake.autoClaw(SolenoidState.OPEN);
        } else if(instance.autoStage == 1) {
            //1 second delay to prevent closing on the cube again >:(
            if(instance.secondaryTimer.get() > 1) {
                //Move to stowed setpoint
                if(ElevFourbar.autoRun(ElevFourbar.STOWED_COORDS)) {
                    //Close the claw and put the pivot down
                    instance.autoStage++;
                }

                if(instance.secondaryTimer.get() > 1.5) {
                    Intake.autoPivot(SolenoidState.DOWN);
                    Intake.runRoller(0); //stop intake
                }
            }
        }
    }

    @Override
    public void print() {
        SmartDashboard.putNumber("Auto Timer 0", timer.get());
        SmartDashboard.putNumber("Auto Timer 1", secondaryTimer.get());
        SmartDashboard.putNumber("Auto Mode", autoMode);
        SmartDashboard.putNumber("Auto Mode Stage", autoStage);
    }
}
    