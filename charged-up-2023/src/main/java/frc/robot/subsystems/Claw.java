package frc.robot.subsystems;

import frc.robot.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Claw {

    static boolean pneumaticsTransitioning;
    static boolean clawClosed;

    public Claw() {
        clawSolenoid = RobotMap.clawSolenoid;

        pneumaticsTransitioning = false;
        clawClosed = false;
    }

    public static void run() {
        if(DriveJoystick.claw() && !pneumaticsTransitioning) {
            pneumaticsTransitioning = true;

            if(RobotMap.clawSolenoid.get() == Value.kForward) {
                RobotMap.clawSolenoid.set(Value.kReverse);
                clawClosed = false;
            } else {
                RobotMap.clawSolenoid.set(Value.kForward);
                clawClosed = true;
            }
        }
        if(!DriveJoystick.claw()) pneumaticsTransitioning = false;
    }

    public static void close() {
        if(RobotMap.clawSolenoid.get() == Value.kForward) {
            RobotMap.clawSolenoid.set(Value.kReverse);
            clawClosed = false;
        }
    }

    public static void release() {
        if(RobotMap.clawSolenoid.get() == Value.kReverse) {
            RobotMap.clawSolenoid.set(Value.kForward);
            clawClosed = true;
        }
    }
}