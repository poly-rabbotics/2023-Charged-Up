
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.systems;

import frc.robot.subsystems.DriveUnit;

public class Drive {
    private static final int LEFT_LEADER_CAN_ID = 0;
    private static final int RIGHT_LEADER_CAN_ID = 1;
    private static final int LEFT_FOLLOWER_CAN_ID = 2;
    private static final int RIGHT_FOLLOWER_CAN_ID = 3;

    private static Drive instance = new Drive();

    private DriveUnit rightUnit;
    private DriveUnit leftUnit;
  
    private Drive() {
        rightUnit = new DriveUnit(RIGHT_LEADER_CAN_ID, RIGHT_FOLLOWER_CAN_ID);
        leftUnit = new DriveUnit(LEFT_LEADER_CAN_ID, LEFT_FOLLOWER_CAN_ID);
    }

    public static void run(double joystickX, double joystickY) {
        instance.rightUnit.set(joystickX + joystickY);
        instance.leftUnit.set(joystickY - joystickX);   
    }
}
