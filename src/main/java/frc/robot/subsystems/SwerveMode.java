// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

public enum SwerveMode {
    /**
     * Moves relative to the driver station, pressing the joystick away from
     * you should always move the robot away from you.
     */
    HEADLESS,

    /**
     * Pressing the joystick forward moves the robot froward, regardless of 
     * position relative to driver.
     */
    RELATIVE,

    /**
     * Rock mode, resists movement and holds position.
     */
    ROCK
}
