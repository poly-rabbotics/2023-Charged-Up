package frc.robot.systems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Fourbar;
import frc.robot.systems.Intake.SolenoidState;

import java.text.DecimalFormat;

public class ElevFourbar {

    //The legnth of the fourbar in inches, used for trig functions
    private static final double FOURBAR_HYPOTENUSE = 37.5;

    //COORDINATE CONSTANTS FOR PID CONTROL
    public static double[] STOWED_COORDS = { 0, FOURBAR_HYPOTENUSE };
    public static double[] GROUND_INTAKE_DOWN_COORDS = { 35.2, 18.2 };
    public static double[] GROUND_INTAKE_UP_COORDS = { 29.5, 5 };
    public static double[] MID_SCORING_COORDS_CONE = { 20.4, 42.5 }; //TODO: verify
    public static double[] MID_SCORING_COORDS_CUBE = { 20.4, 39 };   //TODO: verify
    public static double[] SUBSTATION_INTAKE_COORDS = { 20.4, 40.5 };
    public static double[] HIGH_SCORING_COORDS_CONE = { 26.6, 57.5 }; //TODO: verify
    public static double[] HIGH_SCORING_COORDS_CUBE = { 26.6, 53 };   //TODO: verify

    //enums
    private Setpoint setpoint = Setpoint.STOWED;
    private ControlType controlType = ControlType.POSITION;
    public static GamePiece gamePieceSelected = GamePiece.CONE;

    //deadzone to determine when manual control is enabled
    static final double DEADZONE = 0.3;
    
    //make a decimal format object to improve readability of coordinates
    private static DecimalFormat df = new DecimalFormat("#.###");

    //instantiate coordinates
    private double[] coords = {0, 0};
    private double[] targetCoords = { 0, FOURBAR_HYPOTENUSE };

    private static ElevFourbar instance = new ElevFourbar();

    public static Fourbar fourbar;
    public static Elevator elevator;

    public ElevFourbar() {
        fourbar = new Fourbar();
        elevator = new Elevator();
    }

    public static enum Setpoint {
        SUBSTATION_INTAKE, GROUND_INTAKE, MID_SCORING, HIGH_SCORING, STOWED
    }

    public static enum ControlType {
        MANUAL, POSITION
    }

    public static enum GamePiece {
        CONE, CUBE
    }

    public static void init() {
        instance.controlType = ControlType.POSITION;
        elevator.init();
        fourbar.setPIDSpeed(0.45);
    }

    /**
     * 
     * @param toggleGamePieceMode
     * @param groundIntake
     * @param mid
     * @param high
     * @param stowed
     */
    public static void run(double elevatorSpeed, double fourbarSpeed, int dPadDirection, boolean toggleGamePieceMode, boolean groundIntake, boolean mid, boolean high, boolean stowed, boolean zeroElevEncoder) {
        
        instance.coords = posToCoords(elevator.getPosition(), fourbar.getPosition());

        //toggle between cone and cube mode
        toggleGamePiece(toggleGamePieceMode);

        //set the setpoint depending on which button is pressed
        if(stowed) {
            instance.setpoint = Setpoint.STOWED;
            instance.targetCoords = STOWED_COORDS;
        } else if(groundIntake) {
            instance.setpoint = Setpoint.GROUND_INTAKE;
        } else if(mid) {
            instance.setpoint = Setpoint.MID_SCORING;
            instance.targetCoords = (gamePieceSelected == GamePiece.CONE ? MID_SCORING_COORDS_CONE : MID_SCORING_COORDS_CUBE);
        } else if(high) {
            instance.setpoint = Setpoint.HIGH_SCORING;
            instance.targetCoords = (gamePieceSelected == GamePiece.CONE ? HIGH_SCORING_COORDS_CONE : HIGH_SCORING_COORDS_CUBE);
        } 

        if(instance.setpoint == Setpoint.GROUND_INTAKE) {
            if(Intake.getPivotState() == SolenoidState.UP) {
                instance.targetCoords = GROUND_INTAKE_UP_COORDS;
            } else {
                instance.targetCoords = GROUND_INTAKE_DOWN_COORDS;
            }
        }

        //switches between control modes when button is pressed or manual control detects input
        if(groundIntake || mid || high || stowed) {
            instance.controlType = ControlType.POSITION;
        } else if(Math.abs(elevatorSpeed) > DEADZONE || Math.abs(fourbarSpeed) > DEADZONE) {
            instance.controlType = ControlType.MANUAL;
        } 

        if(instance.controlType == ControlType.POSITION) {
            /* elevator.pidControl(instance.setpoint);
            fourbar.pidControl(instance.setpoint); */

            elevator.pidControl(instance.targetCoords);
            fourbar.pidControl(instance.targetCoords);
        } else {
            elevator.manualControl(elevatorSpeed, dPadDirection);
            fourbar.manualControl(fourbarSpeed);
        }

        if (zeroElevEncoder) {
            elevator.zeroEncoder();
        }
    }
    /**Toggles between cube and cone mode when button pressed
     * @param buttonReleased The button to toggle the game piece with (use a get button released method)
     */
    public static void toggleGamePiece(boolean buttonReleased) {
        gamePieceSelected = (buttonReleased ? (gamePieceSelected == GamePiece.CONE ? GamePiece.CUBE : GamePiece.CONE) : gamePieceSelected);
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
    
    /**
     * Sets elevator and fourbar to desired setpoint
     * @param setpoint The setpoint to run to from the Setpoint enum
     */
    public static boolean autoRun(double[] coords) {

        //Run the fourbar and elevator to inputted setpoint
        elevator.pidControl(coords);
        fourbar.pidControl(coords);

        //return true if the fourbar reached it's destination
        return (Math.abs(fourbar.getPosition() - fourbar.getTargetPosition()) < 1) && (Math.abs(elevator.getPosition() - elevator.getTargetPosition()) < 0.2);
    }
    
    public static void autonomousInit() {
        elevator.autonomousInit();
        fourbar.setPIDSpeed(0.3);
    }


    /**
     * Converts position of the elevator and fourbar to x and y coordinates on a 2d plane
     * @param elevPos The encoder position of the elevator in inches
     * @param fourbarDeg The encoder position of the fourbar in degrees
     * @return [x, y]
     */
    public static double[] posToCoords(double elevPos, double fourbarDeg) {

        double angle = Math.toRadians(90 - fourbarDeg);
        double x;
        double y;

        //funni math stuff my brain hurts
        x = Math.cos(angle) * FOURBAR_HYPOTENUSE;
        y = (Math.sin(angle) * FOURBAR_HYPOTENUSE) + elevPos;

        //return coordinates
        double[] output = { x, y };
        return output;
    }
    
    /**
     * Converts x and y coordinates into encoder positions for the elevator and fourbar
     * @param x
     * @param y
     * @return [elevator position (in), fourbar position (deg)]
     */
    public static double[] coordsToPos(double x, double y) {

        double elevPos;
        double fourbarDeg;

        //calculate elevator position
        elevPos = y >= Math.sqrt((Math.pow(FOURBAR_HYPOTENUSE, 2) - Math.pow(x, 2)))
            ? y - Math.sqrt((Math.pow(FOURBAR_HYPOTENUSE, 2) - Math.pow(x, 2)))
            : y + Math.sqrt((Math.pow(FOURBAR_HYPOTENUSE, 2) - Math.pow(x, 2)));

        //calculate fourbar position
        fourbarDeg = y > elevPos
            ? -Math.toDegrees(Math.atan(x / (elevPos - y)))
            : 180 - Math.toDegrees(Math.atan(x / (elevPos - y)));

        //return positions
        double[] output = { elevPos, fourbarDeg };
        return output;
    }

    public static void updateSmartDashboard(double elevSpeed, double fourbarSpeed) {
        SmartDashboard.putString("Setpoint", instance.setpoint.toString());
        SmartDashboard.putString("Control Type", instance.controlType.toString());

        //fourbar and elevator coordinates
        SmartDashboard.putString("X", df.format(instance.coords[0]));
        SmartDashboard.putString("Y", df.format(instance.coords[1]));
        SmartDashboard.putString("Target Coords", "(" + instance.targetCoords[0] + ", " + instance.targetCoords[1] + ")");

        //Elevator values
        SmartDashboard.putNumber("Elevator Position", elevator.getPosition());
        SmartDashboard.putNumber("Elevator Target", elevator.getTargetPosition()); //ADD THIS
        SmartDashboard.putNumber("Elevator Speed", elevSpeed);

        //Fourbar values
        SmartDashboard.putNumber("Fourbar Position", fourbar.getPosition());
        SmartDashboard.putNumber("Fourbar Target", (fourbar.getTargetPosition() + fourbar.ENCODER_OFFSET)/360.0); //ADD THIS
        SmartDashboard.putNumber("Fourbar Speed", fourbarSpeed);
        SmartDashboard.putNumber("Abs Encoder Position", fourbar.getAbsolutePosition());
    }

}
