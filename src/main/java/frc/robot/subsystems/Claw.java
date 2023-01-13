package frc.robot.subsystems;

import frc.robot.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.*;

public class Claw {

    private DoubleSolenoid clawSolenoid;

    private boolean pneumaticsTransitioning;
    private boolean clawClosed;

    private static Claw instance = new Claw();

    private Claw() {

        pneumaticsTransitioning = false;
        clawClosed = false;

        try {
            clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
        } catch (Exception e) {
            DashboardLog.logWarning("An error occured while initializing clawSolenoid");
		}
    }

    public static void run() {
        if(DriveJoystick.claw() && !instance.pneumaticsTransitioning) {
            instance.pneumaticsTransitioning = true;

            if(instance.clawClosed) {
                instance.clawSolenoid.set(Value.kReverse);
                instance.clawClosed = false;
            } else {
                instance.clawSolenoid.set(Value.kForward);
                instance.clawClosed = true;
            }
        }

        if(!DriveJoystick.claw()) 
            instance.pneumaticsTransitioning = false;
    }

    public static void close() {
        if(RobotMap.clawSolenoid.get() == Value.kForward) {
            instance.clawSolenoid.set(Value.kReverse);
            instance.clawClosed = false;
        }
    }

    public static void release() {
        if(RobotMap.clawSolenoid.get() == Value.kReverse) {
            instance.clawSolenoid.set(Value.kForward);
            instance.clawClosed = true;
        }
    }
}