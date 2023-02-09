package frc.robot.Controls;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveJoystick {
    
    //Sets the drive controller to port zero
    private static XboxController controller = new XboxController(0);

    /**
     * Returns the left joystick's x axis
     */
    public static double getMoveX() {
        return controller.getRawAxis(0);
    }

    /**
     * Returns the left joystick's y axis
     */
    public static double getMoveY() {
        return controller.getRawAxis(1);
    }

    /**
     * Used to activate auto balance
     * @return If right trigger is pressed
     */
    public static boolean getRunAutoBalance() {
        return controller.getRawAxis(3) > 0.4;
    }
}
