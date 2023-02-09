
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.systems;

import frc.robot.subsystems.DriveUnit;
import frc.robot.Controls.*;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.wpilibj.XboxController;

public class Drive {
    private static final int LEFT_LEADER_CAN_ID = 0;
    private static final int RIGHT_LEADER_CAN_ID = 1;
    private static final int LEFT_FOLLOWER_CAN_ID = 2;
    private static final int RIGHT_FOLLOWER_CAN_ID = 3;
    private static final int TILT_DEADZONE = 10;
    private static final int BALANCING_RPM = 300;

    private static Drive instance = new Drive();
    private static XboxController controller = new XboxController(0);

    private DriveUnit rightUnit;
    private DriveUnit leftUnit;
    
    private SparkMaxPIDController rightController;
    private SparkMaxPIDController leftController;

    private static final double P = 0.0000001;
    private static final double I = 0;
    private static final double D = 0;
    private double velocitySetPoint = 0;
    private static boolean isBalancing = false;

    private Drive() {
        rightUnit = new DriveUnit(RIGHT_LEADER_CAN_ID, RIGHT_FOLLOWER_CAN_ID);
        leftUnit = new DriveUnit(LEFT_LEADER_CAN_ID, LEFT_FOLLOWER_CAN_ID);

        rightUnit.settupPID(P, I, D);
        leftUnit.settupPID(P, I, D);

        rightController = rightUnit.getController();
        leftController = leftUnit.getController();
    }

    /**
     * Method to run the drive system during teleop
     */
    public static void run() {

        //gets the x and y axes of the left joystick
        double joystickX = DriveJoystick.getMoveX();
        double joystickY = DriveJoystick.getMoveY();
        
        if(DriveJoystick.getRunAutoBalance()) {
            instance.autoBalance();
            return;
        }

        // rotates joystick axes by 45 degrees & sets units to left and right axes
        instance.rightUnit.set(joystickX + joystickY);
        instance.leftUnit.set(joystickY - joystickX);
    }

    /**
     * Uses PID control to automatically level on charging station, 
     * Detects whether to go forwards or backwards by checking the pitch of the robot
     * and the rate of change of the pitch
     */
    private void autoBalance() {
        if (Pigeon.getRelativePitch() <= -TILT_DEADZONE && Pigeon.getChangePerSecond().pitch > 0.1) { //if robot is tilted forwards
            instance.velocitySetPoint = -BALANCING_RPM;
        } else if (Pigeon.getRelativePitch() >= TILT_DEADZONE && Pigeon.getChangePerSecond().pitch > 0.1) { //if robot is tilted backwards
            instance.velocitySetPoint = BALANCING_RPM;
        } else { //if robot is not tilted
            instance.velocitySetPoint = 0;
        }

        instance.rightController.setReference(velocitySetPoint, ControlType.kVelocity);
        instance.leftController.setReference(-velocitySetPoint, ControlType.kVelocity);

        instance.rightUnit.setController(rightController);
        instance.leftUnit.setController(leftController);
    }
}
