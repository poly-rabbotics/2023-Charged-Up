// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems;

import com.ctre.phoenix.sensors.Pigeon2;

/** Add your docs here. */
public class Pigeon {
    private static final int PIGEON_CAN_ID = 9;

    private static Pigeon instance = new Pigeon(PIGEON_CAN_ID);

    private Pigeon2 pigeon;

    private Pigeon(int canID) {
        pigeon = new Pigeon2(canID);
    }

    public static double getGlobalRotationDegrees() {
        return instance.pigeon.getYaw() % 360.0;
    }
}
