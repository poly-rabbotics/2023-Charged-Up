
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.systems;

import frc.robot.subsystems.DriveUnit;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Drive {
    private static final int LEFT_LEADER_CAN_ID = 0;
    private static final int RIGHT_LEADER_CAN_ID = 1;
    private static final int LEFT_FOLLOWER_CAN_ID = 2;
    private static final int RIGHT_FOLLOWER_CAN_ID = 3;
    private static final int TILT_DEADZONE = 10;
    private static final int BALANCING_RPM = 300;
    private static final int ROCK_MODE_RPM = 300;
    private static final double DRIVE_DEADZONE = 0.3;
    private static final int PANCAKE_FORWARD_CHANNEL_ID = 0; //PLACEHOLDERS, REPLACE LATER
    private static final int PANCAKE_REVERSE_CHANNEL_ID = 1;
    
    private static Drive instance = new Drive();
    
    private DriveUnit rightUnit;
    private DriveUnit leftUnit;
    
    private SparkMaxPIDController rightController;
    private SparkMaxPIDController leftController;
    private DoubleSolenoid gearPancake;
    
    private static final double P = 0.0000001;
    private static final double I = 0;
    private static final double D = 0;
    private double velocitySetPoint = 0;
    
    private boolean highGearActive;
    private boolean isBalancing;
    
    private Drive() {
        rightUnit = new DriveUnit(RIGHT_LEADER_CAN_ID, RIGHT_FOLLOWER_CAN_ID);
        leftUnit = new DriveUnit(LEFT_LEADER_CAN_ID, LEFT_FOLLOWER_CAN_ID);
        
        gearPancake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PANCAKE_FORWARD_CHANNEL_ID, PANCAKE_REVERSE_CHANNEL_ID);
        
        rightUnit.settupPID(P, I, D);
        leftUnit.settupPID(P, I, D);
        
        rightController = rightUnit.getController();
        leftController = leftUnit.getController();
    }
    
    public static void init() {
        instance.isBalancing = false;
        instance.highGearActive = false;
    }
    
    /**
    * Method to run the drive system during teleop
    */
    public static void run(double joystickX, double joystickY, boolean toggleAutoBalance, int dPadDirection, boolean runRockMode, boolean toggleGearShift) {
        
        if(toggleAutoBalance) { //if auto balance button is held
            autoBalance(toggleAutoBalance);
        } else if(runRockMode) { //if rock mode button ins held
            rockMode(joystickX, joystickY);
        } else { //if no special mode is active
            //runs if dpad is pressed
            lowSensitivityMode(dPadDirection);
            
            gearShift(toggleGearShift);
            
            // rotates joystick axes by 45 degrees & sets units to left and right axes
            if(Math.abs(joystickX) > DRIVE_DEADZONE) {
                instance.rightUnit.set(joystickX + joystickY);
                instance.leftUnit.set(joystickY - joystickX);
            }
        }
    }
    
    /**
    * Uses PID control to automatically level on charging station, 
    * Detects whether to go forwards or backwards by checking the pitch of the robot
    * and the rate of change of the pitch
    */
    private static void autoBalance(boolean toggleAutoBalance) {
        if(toggleAutoBalance) {
            if(instance.isBalancing) {
                instance.isBalancing = false;
            } else {
                instance.isBalancing = true;
            }
        }
        
        if(instance.isBalancing) {
            if (Pigeon.getRelativePitch() <= -TILT_DEADZONE && Pigeon.getChangePerSecond().pitch > 0.1) { //if robot is tilted forwards
                instance.velocitySetPoint = -BALANCING_RPM;
            } else if (Pigeon.getRelativePitch() >= TILT_DEADZONE && Pigeon.getChangePerSecond().pitch > 0.1) { //if robot is tilted backwards
                instance.velocitySetPoint = BALANCING_RPM;
            } else { //if robot is not tilted
                instance.velocitySetPoint = 0;
            }
            
            instance.rightController.setReference(instance.velocitySetPoint, ControlType.kVelocity);
            instance.leftController.setReference(instance.velocitySetPoint, ControlType.kVelocity);
            
            instance.rightUnit.setController(instance.rightController);
            instance.leftUnit.setController(instance.leftController);
        }
    }
    
    private static void lowSensitivityMode(int dPadDirection) {
        if(dPadDirection == 0) { //if dPad is pressed up, move forward
            instance.rightUnit.set(0.5);
            instance.leftUnit.set(0.5);
        } else if(dPadDirection == 180) { //if dPad is pressed down, move backward
            instance.rightUnit.set(-0.5);
            instance.leftUnit.set(-0.5);
        } else if(dPadDirection == 90) { //if dPad is pressed right, turn right
            instance.rightUnit.set(-0.5);
            instance.leftUnit.set(0.5);
        } else if(dPadDirection == 270) { //if dPad is pressed left, turn left
            instance.rightUnit.set(0.5);
            instance.leftUnit.set(-0.5);
        } else { //if dPad is not pressed
            instance.rightUnit.set(0);
            instance.leftUnit.set(0);
        }
    }
    
    private static void rockMode(double joystickX, double joystickY) {
        if(Math.abs(joystickX) > DRIVE_DEADZONE) {
            instance.rightController.setReference((joystickX + joystickY) * ROCK_MODE_RPM, ControlType.kVelocity);
            instance.leftController.setReference((joystickY - joystickX) * ROCK_MODE_RPM, ControlType.kVelocity);
        } else {
            instance.rightController.setReference(0, ControlType.kVelocity);
            instance.leftController.setReference(0, ControlType.kVelocity);
        }
    }
    
    private static void gearShift(boolean toggleGearShift) {
        if(toggleGearShift) {
            if(instance.highGearActive) {
                instance.highGearActive = false;
                instance.gearPancake.set(Value.kReverse);
            } else {
                instance.highGearActive = true;
                instance.gearPancake.set(Value.kForward);
            }
        }
    }
}
