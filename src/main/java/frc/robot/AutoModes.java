package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.systems.Intake.SolenoidState;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.systems.*;
import frc.robot.patterns.*;

public class AutoModes {
    //Control panel used for calculating auto mode
    private static Joystick controlPanel = (Joystick)Controls.getControllerByPort(2);
    
    private static Timer timer = new Timer();
    private static Timer secondaryTimer = new Timer();
    public static Timer autoBalanceTimer = new Timer();
    
    private static int autoMode;
    private static int autoStage;
    private static double startTimeBalance = -1.0;

    private static double OUTTAKE_SPEED_CUBE = 0.3;
    private static double OUTTAKE_SPEED_CONE = 0.4;


    static AutoBalanceAlternate autoBalanceAlternate = new AutoBalanceAlternate(6);

    public static int getAutoMode()
    {
        //Calculate auto mode
        int autoMode = 0;
        
        if(controlPanel.getRawButton(12)) 
            autoMode += 1;
        if(controlPanel.getRawButton(11))
            autoMode += 2;
        if(controlPanel.getRawButton(10))
            autoMode += 4;

        return autoMode;
    }
    
    public static void init() {
        autoMode = getAutoMode();
        ElevFourbar.autonomousInit();
        Pigeon.setFeildZero();
        LEDLights.setPatternIfNotEqual(new Breathe(new Color(0.8, 0.3, 0.0), 0.5));
        
        timer.reset();
        timer.start();
        
        secondaryTimer.reset();
        secondaryTimer.start();
        autoStage = 0;

        autoBalanceTimer.reset();
        autoBalanceTimer.start();
    }
    
    public static void run() {

        SmartDashboard.putNumber("Auto Stage", autoStage);

        //Run the auto mode
        switch (autoMode) {
            case 0:
                //Do nothing
                break;
            case 1:
                modeOne();
                break;
            case 2:
                modeTwo();
                break;
            case 3:
                modeThree();
                break;
            case 4:
                modeFour();
                break;
            case 5:
                modeFive();
                break;
            case 6:
                modeSix();
                break;
            case 7:
                modeSeven();
                break;
        }
    }
    
    /**
     * Scores mid then moves out of community
     */
    private static void modeOne() {
        if(autoStage == 0){
            //Move the pivot up
            Intake.autoPivot(SolenoidState.UP);
            ElevFourbar.autoRun((ElevFourbar.gamePieceSelected == ElevFourbar.GamePiece.CUBE) ? ElevFourbar.MID_SCORING_COORDS_CUBE : ElevFourbar.MID_SCORING_COORDS_CONE);
            
            //Move the elevator to the high scoring position
            if(timer.get() > 3) {
                //Open claw when the position has been reached
                secondaryTimer.reset();
                Intake.autoClaw(SolenoidState.OPEN);
                Intake.runRoller((ElevFourbar.gamePieceSelected == ElevFourbar.GamePiece.CUBE) ? OUTTAKE_SPEED_CUBE : OUTTAKE_SPEED_CONE);
                autoStage++;
            } 
            if(timer.get() > 3.1) Intake.autoClaw(SolenoidState.OPEN);
        } else if(autoStage == 1) {
            //1 second delay to prevent closing on the cube again >:(
            if(secondaryTimer.get() > 1) {
                //Move to stowed setpoint
                if(ElevFourbar.autoRun(ElevFourbar.STOWED_COORDS)) {
                    //Close the claw and put the pivot down
                    autoStage++;
                }

                if(secondaryTimer.get() > 1.5) {
                    Intake.autoPivot(SolenoidState.DOWN);
                    Intake.runRoller(0); //stop intake
                }
            }
        }
        
        if (timer.get() > 10 && timer.get() < 15) {
            SwerveDrive.run(0.0, -0.75, 0.0, -1);
        } else {
            SwerveDrive.run(0.0, 0.0, 0.0, -1);
        }
    }

    /**
     * Score only mid
     */
    private static void modeTwo() {
        if(autoStage == 0){
            //Move the pivot up
            Intake.autoPivot(SolenoidState.UP);
            ElevFourbar.autoRun((ElevFourbar.gamePieceSelected == ElevFourbar.GamePiece.CUBE) ? ElevFourbar.MID_SCORING_COORDS_CUBE : ElevFourbar.MID_SCORING_COORDS_CONE);
            
            //Move the elevator to the high scoring position
            if(timer.get() > 3) {
                //Open claw when the position has been reached
                secondaryTimer.reset();
                Intake.autoClaw(SolenoidState.OPEN);
                Intake.runRoller((ElevFourbar.gamePieceSelected == ElevFourbar.GamePiece.CUBE) ? OUTTAKE_SPEED_CUBE : OUTTAKE_SPEED_CONE);
                autoStage++;
            } 
            if(timer.get() > 3.1) Intake.autoClaw(SolenoidState.OPEN);
        } else if(autoStage == 1) {
            //1 second delay to prevent closing on the cube again >:(
            if(secondaryTimer.get() > 1) {
                
                //Move to stowed setpoint
                if(ElevFourbar.autoRun(ElevFourbar.STOWED_COORDS)) {
                    //Close the claw and put the pivot down
                    autoStage++;
                }

                if(secondaryTimer.get() > 1.5) {
                    Intake.autoPivot(SolenoidState.DOWN);
                    Intake.runRoller(0); //stop intake
                }
            }
        }
    }

    /**
     * Score mid then auto balance
     */
    private static void modeThree() {
        if(autoStage == 0){
            //Move the pivot up
            Intake.autoPivot(SolenoidState.UP);
            ElevFourbar.autoRun((ElevFourbar.gamePieceSelected == ElevFourbar.GamePiece.CUBE) ? ElevFourbar.MID_SCORING_COORDS_CUBE : ElevFourbar.MID_SCORING_COORDS_CONE);
            
            //Move the elevator to the high scoring position
            if(timer.get() > 3) {
                //Open claw when the position has been reached
                secondaryTimer.reset();
                Intake.autoClaw(SolenoidState.OPEN);
                Intake.runRoller((ElevFourbar.gamePieceSelected == ElevFourbar.GamePiece.CUBE) ? OUTTAKE_SPEED_CUBE : OUTTAKE_SPEED_CONE);
                autoStage++;
            } 
            if(timer.get() > 3.1) Intake.autoClaw(SolenoidState.OPEN);
        } else if(autoStage == 1) {
            //1 second delay to prevent closing on the cube again >:(
            if(secondaryTimer.get() > 1) {
                //Move to stowed setpoint
                if(ElevFourbar.autoRun(ElevFourbar.STOWED_COORDS)) {
                    //Close the claw and put the pivot down
                    autoStage++;
                }

                if(secondaryTimer.get() > 1.5) {
                    Intake.autoPivot(SolenoidState.DOWN);
                    Intake.runRoller(0); //stop intake
                }
            }
        } else {
            AutoBalance.run();
        }
    }

    /**
     * Score high then move out of the community
     */
    private static void modeFour() {
        if(autoStage == 0){
            //Move the pivot up
            Intake.autoPivot(SolenoidState.UP);
            ElevFourbar.autoRun((ElevFourbar.gamePieceSelected == ElevFourbar.GamePiece.CUBE) ? ElevFourbar.HIGH_SCORING_COORDS_CUBE : ElevFourbar.HIGH_SCORING_COORDS_CONE);
            
            //Move the elevator to the high scoring position
            if(timer.get() > 3) {
                //Open claw when the position has been reached
                secondaryTimer.reset();
                Intake.autoClaw(SolenoidState.OPEN);
                Intake.runRoller((ElevFourbar.gamePieceSelected == ElevFourbar.GamePiece.CUBE) ? OUTTAKE_SPEED_CUBE : OUTTAKE_SPEED_CONE);
                autoStage++;
            } 
            if(timer.get() > 3.1) Intake.autoClaw(SolenoidState.OPEN);
        } else if(autoStage == 1) {
            //1 second delay to prevent closing on the cube again >:(
            if(secondaryTimer.get() > 1) {
                //Move to stowed setpoint
                if(ElevFourbar.autoRun(ElevFourbar.STOWED_COORDS)) {
                    //Close the claw and put the pivot down
                    autoStage++;
                }

                if(secondaryTimer.get() > 1.5) {
                    Intake.autoPivot(SolenoidState.DOWN);
                    Intake.runRoller(0); //stop intake
                }
            }
        }
        
        if (timer.get() > 10 && timer.get() < 15) {
            SwerveDrive.run(0.0, -0.75, 0.0, -1);
        } else {
            SwerveDrive.run(0.0, 0.0, 0.0, -1);
        }
    }

    /**
     * Only score high
     */
    private static void modeFive() {
        if(autoStage == 0){
            //Move the pivot up
            Intake.autoPivot(SolenoidState.UP);
            ElevFourbar.autoRun((ElevFourbar.gamePieceSelected == ElevFourbar.GamePiece.CUBE) ? ElevFourbar.HIGH_SCORING_COORDS_CUBE : ElevFourbar.HIGH_SCORING_COORDS_CONE);
            
            //Move the elevator to the high scoring position
            if(timer.get() > 3) {
                //Open claw when the position has been reached
                secondaryTimer.reset();
                Intake.runRoller((ElevFourbar.gamePieceSelected == ElevFourbar.GamePiece.CUBE) ? OUTTAKE_SPEED_CUBE : OUTTAKE_SPEED_CONE); //run intake out
                autoStage++;
            }
            if(timer.get() > 3.1) Intake.autoClaw(SolenoidState.OPEN);
        } else if(autoStage == 1) {
            //1 second delay to prevent closing on the cube again >:(
            if(secondaryTimer.get() > 1) {
                //Move to stowed setpoint
                if(ElevFourbar.autoRun(ElevFourbar.STOWED_COORDS)) {
                    //Close the claw and put the pivot down
                    autoStage++;
                }

                if(secondaryTimer.get() > 1.5) {
                    Intake.autoPivot(SolenoidState.DOWN);
                    Intake.runRoller(0); //stop intake
                }
            }
        }
    }

    /**
     * Score high and auto balance
     */
    private static void modeSix() {
        if(autoStage == 0){
            //Move the pivot up
            Intake.autoPivot(SolenoidState.UP);
            ElevFourbar.autoRun((ElevFourbar.gamePieceSelected == ElevFourbar.GamePiece.CUBE) ? ElevFourbar.HIGH_SCORING_COORDS_CUBE : ElevFourbar.HIGH_SCORING_COORDS_CONE);
            
            //Move the elevator to the high scoring position
            if(timer.get() > 3) {
                //Open claw when the position has been reached
                secondaryTimer.reset();
                Intake.autoClaw(SolenoidState.OPEN);
                Intake.runRoller((ElevFourbar.gamePieceSelected == ElevFourbar.GamePiece.CUBE) ? OUTTAKE_SPEED_CUBE : OUTTAKE_SPEED_CONE);
                autoStage++;
            } 
            if(timer.get() > 3.1) Intake.autoClaw(SolenoidState.OPEN);
        } else if(autoStage == 1) {
            //1 second delay to prevent closing on the cube again >:(
            if(secondaryTimer.get() > 1) {
                //Move to stowed setpoint
                if(ElevFourbar.autoRun(ElevFourbar.STOWED_COORDS)) {
                    //Close the claw and put the pivot down
                    autoStage++;
                }

                if(secondaryTimer.get() > 1.5) {
                    Intake.autoPivot(SolenoidState.DOWN);
                    Intake.runRoller(0); //stop intake
                }
            }
        } else {
            AutoBalance.run();
        }
    }

    /**
     * Only drop game piece and move out of the community
     */
    private static void modeSeven() {
        Intake.autoPivot(SolenoidState.UP);
        Intake.autoClaw(SolenoidState.OPEN);
        Intake.runRoller(0.8);
        //autoBalanceAlternate.run();
        if (timer.get() > 10 && timer.get() < 15) {
            SwerveDrive.run(0.0, -0.75, 0.0, -1);
        } else {
            SwerveDrive.run(0.0, 0.0, 0.0, -1);
        }
    }

    private static void autoBalance(double startTime) {
        if (timer.get() > startTime) {
            if (startTimeBalance == -1.0) {
                startTimeBalance = timer.get();
            }

            if (timer.get() - startTimeBalance >= 1.5) {
                SwerveDrive.run(0.0, 0.0, 0.0, -1);
                return;
            }

            /* if (false) {
                startTimeBalance = -1.0;
                if (timer.get() > 0 && timer.get() < 15) {
                    SwerveDrive.run(0.0, -0.85, 0.0, -1);
                } else {
                    SwerveDrive.run(0.0, 0.0, 0.0, -1);
                }
            } */
        }
    }
}
