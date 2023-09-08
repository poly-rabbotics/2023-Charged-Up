package frc.robot.systems;

import java.text.DecimalFormat;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.systems.Intake.SolenoidState;
import frc.robot.subsystems.DoubleSetpoint;
import frc.robot.subsystems.Coordinate;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Setpoint;
import frc.robot.subsystems.Fourbar;
import frc.robot.SmartPrintable;

public class ElevFourbar extends SmartPrintable {

    //SETPOINTS FOR PID CONTROL
    public static final DoubleSetpoint STOWED_SETPOINT = new DoubleSetpoint(new Setpoint(0, Setpoint.HYPOTENUSE), new Setpoint(0, Setpoint.HYPOTENUSE));
    public static final DoubleSetpoint MID_SCOORING_SETPOINT = new DoubleSetpoint(new Setpoint(15, 37), new Setpoint(15, 34.4));
    public static final DoubleSetpoint HIGH_SCORING_SETPOINT = new DoubleSetpoint(new Setpoint(26.6, 50), new Setpoint(33.2, 48.6));

    public static final Setpoint GROUND_INTAKE_DOWN_SETPOINT = new Setpoint(35.2, 11, true);
    public static final Setpoint GROUND_INTAKE_UP_SETPOINT = new Setpoint(30, 1, true);
    public static final Setpoint SUBSTATION_INTAKE_SETPOINT = new Setpoint(20.4, 40.5);

    //deadzone to determine when manual control is enabled
    static final double DEADZONE = 0.3;
    
    //make a decimal format object to improve readability of coordinates
    private static DecimalFormat df = new DecimalFormat("#.###");

    //Instance veriables
    private Setpoint setpoint;
    private Setpoint currentSetpoint;
    private ControlType controlType = ControlType.POSITION;
    private GamePiece gamePieceSelected = GamePiece.CUBE;
    private boolean translateMode = false;

    //Elevator and Fourbar objects
    public static Fourbar fourbar;
    public static Elevator elevator;

    //Instance of this class
    private static ElevFourbar instance = new ElevFourbar();

    public static enum SetpointEnum {
        SUBSTATION_INTAKE, GROUND_INTAKE, MID_SCORING, HIGH_SCORING, STOWED
    }

    public static enum ControlType {
        MANUAL, POSITION
    }

    public static enum GamePiece {
        CONE, CUBE
    }

    /**
     * Literally just a constructor
     */
    public ElevFourbar() {
        super();
        fourbar = new Fourbar();
        elevator = new Elevator();
        currentSetpoint = new Setpoint(0, Setpoint.HYPOTENUSE);
        setpoint = STOWED_SETPOINT.cone;
    }

    /**
     * Initialize teleop
     */
    public static void init() {
        instance.controlType = ControlType.POSITION;
        elevator.init();
        fourbar.setPIDSpeed(0.45);
        instance.translateMode = false;

        instance.setpoint = getGamePieceSelected() == GamePiece.CUBE ? STOWED_SETPOINT.cube : STOWED_SETPOINT.cone;
    }
    
    /**
     * Initialize autonomous
     */
    public static void autonomousInit() {
        elevator.autonomousInit();
        fourbar.setPIDSpeed(0.3);
    }

    public static void run(double elevatorSpeed, double fourbarSpeed, int dPadDirection, boolean toggleGamePieceMode, boolean groundIntake, boolean mid, boolean high, boolean zeroElevEncoder) {
        instance.currentSetpoint = new Setpoint(false, elevator.getPosition(), fourbar.getPosition());

        //toggle between cone and cube mode
        toggleGamePiece(toggleGamePieceMode);

        //set the target setpoint based on which button is pressed
        setSetPoint(groundIntake, mid, high);

        //switches between control modes when button is pressed or manual control detects input
        if(Math.abs(elevatorSpeed) > DEADZONE || Math.abs(fourbarSpeed) > DEADZONE) {
            instance.controlType = ControlType.MANUAL;
        } else if(mid || groundIntake || high) {
            instance.controlType = ControlType.POSITION;
        } 

        if(instance.controlType == ControlType.POSITION) { //Run PID control
            elevator.pidControl(instance.setpoint);
            fourbar.pidControl(instance.setpoint);
        } else { //Run manual control
            if(instance.translateMode) {

            } else {

                elevator.manualControl(elevatorSpeed, dPadDirection);
                fourbar.manualControl(fourbarSpeed);

            }
        }


        if (zeroElevEncoder) {
            elevator.zeroEncoder();
        }

        SmartDashboard.putNumber("Fourbar manual", fourbarSpeed);
        SmartDashboard.putNumber("Elevator manual", elevatorSpeed);
        SmartDashboard.putString("CT", instance.controlType.toString());
    }

    /**
     * Set the current setpoint based on which button is held
     * @param ground
     * @param mid
     * @param high
     */
    public static void setSetPoint(boolean ground, boolean mid, boolean high) {

        if(ground) {
            instance.setpoint = Intake.getPivotState() == SolenoidState.DOWN 
                ? GROUND_INTAKE_DOWN_SETPOINT 
                : GROUND_INTAKE_UP_SETPOINT;
        } else if(mid) {
            instance.setpoint = getGamePieceSelected() == GamePiece.CUBE 
                ? MID_SCOORING_SETPOINT.cube 
                : MID_SCOORING_SETPOINT.cone;
        } else if(high) {
            instance.setpoint = getGamePieceSelected() == GamePiece.CUBE 
                ? HIGH_SCORING_SETPOINT.cube 
                : HIGH_SCORING_SETPOINT.cone;
        } else {
            instance.setpoint = getGamePieceSelected() == GamePiece.CUBE
                ? STOWED_SETPOINT.cube 
                : STOWED_SETPOINT.cone;
        }
    }

    /**
     * Sets elevator and fourbar to desired setpoint
     * @param setpoint The setpoint to run to from the Setpoint enum
     */
    public static boolean autoRun(Setpoint setpoint) {

        //Run the fourbar and elevator to inputted setpoint
        elevator.pidControl(setpoint);
        fourbar.pidControl(setpoint);

        //return true if the fourbar reached it's destination
        return (Math.abs(fourbar.getPosition() - fourbar.getTargetPosition()) < 1) && (Math.abs(elevator.getPosition() - elevator.getTargetPosition()) < 0.2);
    }

    
    /**Toggles between cube and cone mode when button pressed
     * @param buttonReleased The button to toggle the game piece with (use a get button released method)
     */
    public static void toggleGamePiece(boolean buttonReleased) {
        instance.gamePieceSelected = (buttonReleased ? (instance.gamePieceSelected == GamePiece.CONE ? GamePiece.CUBE : GamePiece.CONE) : instance.gamePieceSelected);
    }

    public static Coordinate getCurrentPos() {
        return instance.currentSetpoint.getCoords();
    }

    public static Setpoint getSetpoint() {
        return instance.setpoint;
    }

    public static GamePiece getGamePieceSelected() {
        return instance.gamePieceSelected;
    }

    public static ControlType getControlType() {
        return instance.controlType;
    }

    public static void setFourbarBrake(boolean brake) {
        fourbar.setBrake(brake);
    }

    public static void toggleTranslateMode() {
        instance.translateMode = !instance.translateMode;
    }

    public void print() {
        SmartDashboard.putString("Control Type", instance.controlType.toString());

        //fourbar and elevator coordinates
        SmartDashboard.putString("Current coords", "(" + getCurrentPos().x + ", " + getCurrentPos().y + ")");
        SmartDashboard.putString("Target Coords", "(" + instance.setpoint.getCoords().x + ", " + instance.setpoint.getCoords().y + ")");
        SmartDashboard.putNumber("Bumper Intercept", fourbar.getBumperIntercept());
        SmartDashboard.putNumber("Fourbar Slope", fourbar.getSlope());

        //Elevator values
        SmartDashboard.putNumber("Elevator Position", elevator.getPosition());
        SmartDashboard.putNumber("Elevator Target", elevator.getTargetPosition()); //ADD THIS

        //Fourbar values
        SmartDashboard.putNumber("Fourbar Position", fourbar.getPosition());
        SmartDashboard.putNumber("Fourbar Target", (fourbar.getTargetPosition() + fourbar.ENCODER_OFFSET)); //ADD THIS
        SmartDashboard.putNumber("Abs Encoder Position", fourbar.getAbsolutePosition());

        SmartDashboard.putBoolean("Cube Mode Selected?", getGamePieceSelected() == ElevFourbar.GamePiece.CUBE);
        SmartDashboard.putBoolean("Cone Mode Selected?", getGamePieceSelected() == ElevFourbar.GamePiece.CONE);
    }
}
