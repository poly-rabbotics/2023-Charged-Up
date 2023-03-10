package frc.robot.systems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Fourbar;

public class ElevFourbar {
    private Setpoint setpoint = Setpoint.STOWED;
    private ControlType controlType = ControlType.POSITION;
    private boolean manualControl = false;
    static final double DEADZONE = 0.3;

    private static ElevFourbar instance = new ElevFourbar();

    public ElevFourbar() {

    }

    public static enum Setpoint {
        SUBSTATION_INTAKE, GROUND_INTAKE, MID_SCORING, HIGH_SCORING, STOWED
    }

    public static enum ControlType {
        MANUAL, POSITION
    }

    public static void init() {
        instance.controlType = ControlType.POSITION;
        Elevator.init();
    }

    /**
     * 
     * @param substationIntake
     * @param groundIntake
     * @param mid
     * @param high
     * @param stowed
     */
    public static void run(double elevatorSpeed, double fourbarSpeed, boolean elevatorResetEncoder, boolean fourbarResetEncoder, boolean runAutoCalibrate, int dPadDirection, boolean substationIntake, boolean groundIntake, boolean mid, boolean high, boolean stowed, boolean switchControlType) {
        
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

        if(substationIntake || groundIntake || mid || high || stowed) {
            instance.controlType = ControlType.POSITION;
        }

        /* //switches between control modes
        if(switchControlType) {
            if(instance.controlType == ControlType.MANUAL) {
                instance.controlType = ControlType.POSITION;
            } else {
                instance.controlType = ControlType.MANUAL;
            }
        } */

        if(Math.abs(elevatorSpeed) > DEADZONE || Math.abs(fourbarSpeed) > DEADZONE) {
            instance.controlType = ControlType.MANUAL;
        } 

        //runs selected control mode
        Elevator.run(
            elevatorSpeed,
            elevatorResetEncoder,
            runAutoCalibrate,
            dPadDirection,
            instance.setpoint,
            instance.controlType
        );

        Fourbar.run(
            fourbarSpeed,
            fourbarResetEncoder,
            instance.setpoint,
            instance.controlType
        ); 
        
        SmartDashboard.putString("Setpoint", instance.setpoint.toString());
        SmartDashboard.putString("Control Type", instance.controlType.toString());
    }

    public static boolean autoRun(Setpoint setpoint) {
        Elevator.run(
            0, 
            false, 
            false, 
            0, 
            setpoint, 
            ControlType.POSITION
        );

        Fourbar.run(0, false, setpoint, ControlType.POSITION);

        return Math.abs(Fourbar.getPosition() - Fourbar.getTargetSetpoint()) < 1;
    }

    public static void autonomousRun(Setpoint setpoint) {
        Elevator.autonomousRun(setpoint);
        Fourbar.autonomousRun(setpoint);
    }

    public static void autonomousInit() {
        Elevator.autonomousInit();
    }

    public static boolean getIsFinished() {
        return Elevator.getIsFinished() && Fourbar.getIsFinished();
    }

}
