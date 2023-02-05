// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems;

import com.ctre.phoenix.sensors.Pigeon2;

public class Pigeon {
    private static final int PIGEON_CAN_ID = 9;

    private static Pigeon instance = new Pigeon(PIGEON_CAN_ID);

    private Pigeon2 pigeon;
    private double relativeForward = 0.0;

    private Pigeon(int canID) {
        pigeon = new Pigeon2(canID);
    }

    /**
     * Sets "relative forward" to the current position. This will make getting
     * the relative rotation always return a rotation relative to the rotation
     * the robot is in at the point this method is called.
     */
    public static void setRelativeForward() {
        instance.relativeForward = getGobalRotationDegrees();
    }

    /**
     * Gets the robot's rotation in respect to relative forward, if relative
     * forward has not been set then it simply returns the absolute rotaton.
     */
    public static double getRelativeRotationDegrees() {
        return (getGlobalRotationDegrees() - instance.relativeForward) % 360.0;
    }

    public static double getGlobalRotationDegrees() {
        return instance.pigeon.getYaw() % 360.0;
    }
}
