package frc.robot;

public class RobotMap {
    //ALL HARDWARE IDs OR PORTS GO HERE

    //S W E R V E   D R I V E 
    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 1;
    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 2;
    public static final int BACK_LEFT_DRIVE_MOTOR_ID = 3;
    public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 4;

    public static final int FRONT_RIGHT_ROTATION_MOTOR_ID = 5;
    public static final int FRONT_LEFT_ROTATION_MOTOR_ID = 6;
    public static final int BACK_LEFT_ROTATION_MOTOR_ID = 7;
    public static final int BACK_RIGHT_ROTATION_MOTOR_ID = 8;

    public static final int FRONT_LEFT_CANCODER_ID = 9;
    public static final int BACK_LEFT_CANCODER_ID = 10; 
    public static final int FRONT_RIGHT_CANCODER_ID = 11;
    public static final int BACK_RIGHT_CANCODER_ID = 12;

    //E L E V A T O R / F O U R B A R
    public static final int ELEVATOR_MOTOR_ID = 62; //TODO: CHANGE THIS
    public static final int FOURBAR_MOTOR_ID = 2; //TODO: CHANGE THIS

    //I N T A K E 
    public static final int INTAKE_MOTOR_ID = 0;
    public static final int[] INTAKE_PIVOT_IDS = {8,9}; //TODO: CHANGE THESE
    public static final int[] INTAKE_CLAW_IDS = {10,11}; //TODO: CHANGE THESE


}
