package frc.robot;

import java.util.concurrent.*;
import edu.wpi.first.wpilibj.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class RobotMap {
	// NOTE: this should be the only instance of this class, ever.
	// It only works if you use the same one everywhere.

        /* --> Joysticks <-- */
    public static XboxController driveJoystick;

        /* --> Intake <-- */
    public static DoubleSolenoid clawSolenoid;
    public static CANSparkMax deleteThisLater = new CANSparkMax(2, MotorType.kBrushless);

    public static void initJoysticks(){
        try{
            driveJoystick = new XboxController(0);
        } catch (Exception e) {
        	
		} 
    }

    public static void initClaw() {
        try {
            clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
        } catch (Exception e) {
        	
		}
    }
}