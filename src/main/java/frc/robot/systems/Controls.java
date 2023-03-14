// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class Controls {
    public static XboxController controllerOne;// = new XboxController(0);
    public static XboxController controllerTwo;// = new XboxController(1);
    public static Joystick controlPanel;// = new Joystick(2);
    
    private static final Controls instance = new Controls();

    private Controls() {

    }
}
