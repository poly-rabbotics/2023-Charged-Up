package frc.robot.systems;

import java.security.InvalidParameterException;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.*;

import frc.robot.systems.ElevFourbar.GamePiece;
import frc.robot.systems.Intake.SolenoidState;
import frc.robot.subsystems.DoubleSetpoint;
import frc.robot.subsystems.Setpoint;
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
        AutonomousRunner::modeSeven
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
     * Only move out of the community
     */
    private static void modeOne() {
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
        score(ElevFourbar.MID_SCOORING_SETPOINT);
    }

    /**
     * Scores mid then moves out of community
     */
    private static void modeThree() {

        score(ElevFourbar.MID_SCOORING_SETPOINT);
        
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
     * Score mid then auto balance
     */
    private static void modeFour() {

        score(ElevFourbar.MID_SCOORING_SETPOINT);
        
        if (instance.autoStage > 1 || instance.timer.get() > 6) {
            AutoBalance.run();
        }
    }

    /**
     * Only score high
     */
    private static void modeFive() {
        score(ElevFourbar.HIGH_SCORING_SETPOINT);
    }

    /**
     * Score high then move out of the community
     */
    private static void modeSix() {

        score(ElevFourbar.HIGH_SCORING_SETPOINT);

        
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
     * Score high and auto balance
     */
    private static void modeSeven() {
        score(ElevFourbar.HIGH_SCORING_SETPOINT);
        
        if (instance.autoStage > 1 || instance.timer.get() > 6) {
            AutoBalance.run();
        }
    }

    private static void score(DoubleSetpoint setpoint) {
        if(instance.autoStage == 0){
            //Move the pivot up
            Intake.autoPivot(SolenoidState.UP);
            
            ElevFourbar.setSetpointAuto((ElevFourbar.getGamePieceSelected() == GamePiece.CUBE ? setpoint.cube : setpoint.cone));
            ElevFourbar.autoRun();

            //Move the elevator to the high scoring position
            if(instance.timer.get() > 1.5) {
                //Open claw when the position has been reached
                Intake.autoClaw(SolenoidState.OPEN);
                Intake.runRoller((ElevFourbar.getGamePieceSelected() == ElevFourbar.GamePiece.CUBE) ? OUTTAKE_SPEED_CUBE : OUTTAKE_SPEED_CONE); //run intake out
            }
            
            if(instance.timer.get() > 2.5) {
                instance.autoStage++;
            }
        } else if(instance.autoStage == 1) {
            //1 second delay to prevent closing on the cube again >:(
            if(instance.secondaryTimer.get() > 1) {
                ElevFourbar.setSetpointAuto(new Setpoint(0, Setpoint.HYPOTENUSE));
                ElevFourbar.autoRun();

                //Move to stowed setpoint
                if(instance.secondaryTimer.get() > 2) {
                    //Close the claw and put the pivot down
                    instance.autoStage++;
                }

                if(instance.secondaryTimer.get() > 2.5) {
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
    