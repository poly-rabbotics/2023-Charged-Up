
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.systems;

import frc.robot.subsystems.DriveUnit;
import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

public class Drive {
    private static final int LEFT_LEADER_CAN_ID = 0;
    private static final int RIGHT_LEADER_CAN_ID = 1;
    private static final int LEFT_FOLLOWER_CAN_ID = 2;
    private static final int RIGHT_FOLLOWER_CAN_ID = 3;
    private static final int PIGEON_CAN_ID = 9;
    private static final int PID_DEADZONE = 5;

    private static Drive instance = new Drive();

    private DriveUnit rightUnit;
    private DriveUnit leftUnit;

    private Pigeon2 pigeon;

    private static final double P = 0;
    private static final double I = 0;
    private static final double D = 0;
    private double velocitySetPoint = 0;
    private boolean isBalancing = false;

    private Drive() {
        rightUnit = new DriveUnit(RIGHT_LEADER_CAN_ID, RIGHT_FOLLOWER_CAN_ID);
        leftUnit = new DriveUnit(LEFT_LEADER_CAN_ID, LEFT_FOLLOWER_CAN_ID);

        rightUnit.settupPID(P, I, D);
        rightUnit.settupPID(P, I, D);

        pigeon = new Pigeon2(PIGEON_CAN_ID);
    }

    public static void run(double joystickX, double joystickY) {
        if(instance.isBalancing) {
            instance.autoBalance();
            return;
        }

        // rotates joystick axes by 45 degrees & sets units to left and right axes
        instance.rightUnit.set(joystickX + joystickY);
        instance.leftUnit.set(joystickY - joystickX);
    }

    private void autoBalance() {
        if (instance.getGlobalRotation() <= -PID_DEADZONE) { //if robot is tilted forwards
            instance.velocitySetPoint = -30;
        } else if (instance.getGlobalRotation() >= PID_DEADZONE) { //if robot is tilted backwards
            instance.velocitySetPoint = 30;
        } else { //if robot is not tilted
            instance.velocitySetPoint = 0;
        }

        instance.rightController.setReference(velocitySetPoint, ControlType.kVelocity);
        instance.leftController.setReference(velocitySetPoint, ControlType.kVelocity);

        instance.rightUnit.setController(rightController);
        instance.leftUnit.setController(leftController);
    }

    private double getGlobalRotation() {
        return instance.pigeon.getPitch() % 360.0;
    }
}
