package frc.robot;

import java.util.concurrent.*;
import edu.wpi.first.wpilibj.*;

public class RobotMap {

        /* --> Joysticks <-- */
    public static XboxController driveJoystick;

        /* --> Intake <-- */
    public static DoubleSolenoid clawSolenoid;

    public static void initJoysticks(){
        try{
            driveJoystick = new XboxController(0);
        } catch (Exception e) {
        	dashboardLog.logError("Error occured while initializing joysticks.");
			dashboardLog.logError(e);
		} 
    }

    public static void initClaw() {
        try {
            clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 5);
        } catch (Exception e) {
        	dashboardLog.logError("Error occured while initializing claw.");
			dashboardLog.logError(e);
		}
    }
}