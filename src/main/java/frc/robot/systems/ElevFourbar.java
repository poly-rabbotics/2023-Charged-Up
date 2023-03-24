package frc.robot.systems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Fourbar;

public class ElevFourbar {
    private Setpoint setpoint = Setpoint.STOWED;
    private ControlType controlType = ControlType.POSITION;
    static final double DEADZONE = 0.3;

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
        double fourbarPosRadians = Math.toRadians(90 - fourbar.getPosition());

        double x;
        double y;
        x = Math.cos(fourbarPosRadians) * 39.5;
        y = (Math.sin(fourbarPosRadians) * 39.5) + elevator.getPosition();
        SmartDashboard.putNumber("X", x);
        SmartDashboard.putNumber("Y", y);

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
        return (Math.abs(fourbar.getPosition() - fourbar.getTargetPosition()) < 1) && (Math.abs(elevator.getPosition() - elevator.getTargetPosition()) < 1);
    }

    public static void autonomousInit() {
        elevator.autonomousInit();
    }



    private static void updateSmartDashboard(double elevSpeed, double fourbarSpeed) {
        SmartDashboard.putString("Setpoint", instance.setpoint.toString());
        SmartDashboard.putString("Control Type", instance.controlType.toString());

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
