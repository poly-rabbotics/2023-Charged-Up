package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;


public class DriveJoystick {
    public static XboxController joystick = RobotMap.driveJoystick;

    public static boolean claw() { //Closes and opens claw
        return joystick.getRawButton(1); //A button
    }
}