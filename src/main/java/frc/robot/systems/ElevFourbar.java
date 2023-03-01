package frc.robot.systems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Fourbar;

public class ElevFourbar {
    private Setpoint setpoint = Setpoint.STOWED;
    private ControlType controlType = ControlType.MANUAL;

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
        instance.setpoint = Setpoint.STOWED;
        instance.controlType = ControlType.MANUAL;

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

        //switches between control modes
        if(switchControlType) {
            if(instance.controlType == ControlType.MANUAL) {
                instance.controlType = ControlType.POSITION;
            } else {
                instance.controlType = ControlType.MANUAL;
            }
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


}
