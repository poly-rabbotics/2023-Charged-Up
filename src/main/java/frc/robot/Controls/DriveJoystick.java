package frc.robot.Controls;

import edu.wpi.first.wpilibj.XboxController;

public class DriveJoystick {
    private static final int JOYSTICK_PORT = 0;

    private static DriveJoystick instance = new DriveJoystick(JOYSTICK_PORT);
    private XboxController joystick;

    private DriveJoystick(int port) {
        joystick = new XboxController(port);
    }

    public static double getMove() {
        return instance.joystick.getRawAxis(1);
    }

    public static double getRotate() {
        return instance.joystick.getRawAxis(4);
    }

    public static double getAxis0() {
        return instance.joystick.getRawAxis(0);
    }
    public static boolean getToggleMode() {
        return instance.joystick.getRawButton(1);
    }



}
