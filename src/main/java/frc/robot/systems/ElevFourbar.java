package frc.robot.systems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Fourbar;
import java.text.DecimalFormat;

public class ElevFourbar {
    private Setpoint setpoint = Setpoint.STOWED;
    private ControlType controlType = ControlType.POSITION;

    static final double DEADZONE = 0.3;
    
    //make a decimal format object to improve readability of coordinates
    private static DecimalFormat df = new DecimalFormat("#.###");

    //The legnth of the fourbar in inches
    private static final double FOURBAR_HYPOTENUSE = 37.5;

    private double[] coords = { 0, 0 };

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

    public static void init() {
        instance.controlType = ControlType.POSITION;
        elevator.init();
    }

    /**
     * 
     * @param substationIntake
     * @param groundIntake
     * @param mid
     * @param high
     * @param stowed
     */
    public static void run(double elevatorSpeed, double fourbarSpeed, int dPadDirection, boolean substationIntake, boolean groundIntake, boolean mid, boolean high, boolean stowed) {
        
        //kyle learning trig stuff
        instance.coords = posToCoords(elevator.getPosition(), fourbar.getPosition());


        //set the setpoint depending on which button is pressed
        if(substationIntake) {
            instance.setpoint = Setpoint.SUBSTATION_INTAKE;
        } else if(groundIntake) {
            instance.setpoint = Setpoint.GROUND_INTAKE;
        } else if(mid) {
            instance.setpoint = Setpoint.MID_SCORING;
        } else if(high) {
            instance.setpoint = Setpoint.HIGH_SCORING;
        } else if(stowed) {
            instance.setpoint = Setpoint.STOWED;
        }

        //switches between control modes when button is pressed or manual control detects input
        if(substationIntake || groundIntake || mid || high || stowed) {
            instance.controlType = ControlType.POSITION;
        } else if(Math.abs(elevatorSpeed) > DEADZONE || Math.abs(fourbarSpeed) > DEADZONE) {
            instance.controlType = ControlType.MANUAL;
        } 

        //runs selected control mode
        /* Elevator.run(
            elevatorSpeed,
            elevatorResetEncoder,
            dPadDirection,
            instance.setpoint,
            instance.controlType
        );

        fourbar.run(
            fourbarSpeed,
            fourbarResetEncoder,
            instance.setpoint,
            instance.controlType
        );  */

        if(instance.controlType == ControlType.POSITION) {
            elevator.pidControl(instance.setpoint);
            fourbar.pidControl(instance.setpoint);
        } else {
            elevator.manualControl(elevatorSpeed, dPadDirection);
            fourbar.manualControl(-fourbarSpeed);
        }
        
        updateSmartDashboard(elevatorSpeed, fourbarSpeed);
    }

    public static boolean autoRun(Setpoint setpoint) {

        //Run the fourbar and elevator to inputted setpoint
        elevator.pidControl(setpoint);
        fourbar.pidControl(setpoint);

        //return true if the fourbar reached it's destination
        return (Math.abs(fourbar.getPosition() - fourbar.getTargetPosition()) < 1) && (Math.abs(elevator.getPosition() - elevator.getTargetPosition()) > 0.2);
    }

    public static void autonomousRun(Setpoint setpoint) {
        elevator.autonomousRun(setpoint);
        fourbar.autonomousRun(setpoint);
    }

    public static void autonomousInit() {
        elevator.autonomousInit();
    }


    /**
     * Converts position of the elevator and fourbar to x and y coordinates on a 2d plane
     * @param elevPos The encoder position of the elevator in inches
     * @param fourbarDeg The encoder position of the fourbar in degrees
     * @return [x, y]
     */
    private static double[] posToCoords(double elevPos, double fourbarDeg) {

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
    private static double[] coordsToPos(double x, double y) {

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

    private static void updateSmartDashboard(double elevSpeed, double fourbarSpeed) {
        SmartDashboard.putString("Setpoint", instance.setpoint.toString());
        SmartDashboard.putString("Control Type", instance.controlType.toString());

        //fourbar and elevator coordinates
        SmartDashboard.putString("X", df.format(instance.coords[0]));
        SmartDashboard.putString("Y", df.format(instance.coords[1]));

        //Elevator values
        SmartDashboard.putNumber("Elevator Position", elevator.getPosition());
        SmartDashboard.putNumber("Elevator Target", elevator.getTargetPosition()); //ADD THIS
        SmartDashboard.putNumber("Elevator Speed", elevSpeed);

        //Fourbar values
        SmartDashboard.putNumber("Fourbar Position", fourbar.getPosition());
        SmartDashboard.putNumber("Fourbar Target", fourbar.getTargetPosition()); //ADD THIS
        SmartDashboard.putNumber("Fourbar Speed", fourbarSpeed);
    }

}
