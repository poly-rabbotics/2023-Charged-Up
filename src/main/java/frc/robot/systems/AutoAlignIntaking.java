package frc.robot.systems;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Limelight;

public class AutoAlignIntaking {
    
    public static AutoAlignIntaking instance = new AutoAlignIntaking();

    public static boolean autoAligning = false;
    
    private double kP_X, kI_X, kD_X, kP_Y, kI_Y, kD_Y;

    private static PIDController pidX;
    private static PIDController pidY;

    private static double driveSpeedX, driveSpeedY;

    private static double xDisplacement, yDisplacement;
    private static double TOLERANCE_X = 3; //degrees
    private static double TOLERANCE_Y = 3; //inches

    private static final double INTAKE_LENGTH = 16; //inches //TODO: Tune this value

    public AutoAlignIntaking() {
        //TODO: Tune PID values (these are completely untested)
        kP_X = 0.0001;
        kI_X = 0.0000000;
        kD_X = 0.0000000;

        kP_Y = 0.0001;
        kI_Y = 0.0000000;
        kD_Y = 0.0000000;

        pidX = new PIDController(kP_X, kI_X, kD_X);

        pidY = new PIDController(kP_Y, kI_Y, kD_Y);

        driveSpeedX = driveSpeedY = 0;
        xDisplacement = yDisplacement = 0;
    }
    
    /**
     * Run the auto align sequence as long as button is held
     * @param holdToAlign button from drive joystick to hold
     */
    public static void run(boolean holdToAlign) {
        xDisplacement = Limelight.getDegreesOffsetX();
        yDisplacement = Sonar.getDistance() - INTAKE_LENGTH;
        
        driveSpeedX = pidX.calculate(xDisplacement);
        driveSpeedY = pidY.calculate(yDisplacement);
        
        if (holdToAlign) {
            autoAligning = true;
            SwerveDrive.runUncurved(driveSpeedX, -driveSpeedY, 0); //y speed negated because claw side forward 
        } else autoAligning = false;  
        
    }

    public static boolean isAlignedX() {
        return Math.abs(xDisplacement) < TOLERANCE_X;
    }

    public static boolean isAlignedY() {
        return Math.abs(yDisplacement) < TOLERANCE_Y;
    }

    public static boolean isAligned() {
        return isAlignedX() && isAlignedY();
    }
}
