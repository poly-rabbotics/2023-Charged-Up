// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class Fourbar {
    private static XboxController joystick = new XboxController(0); //temporary pidController, will be replaced with the universal pidControllers

    private static final int FOURBAR_UPPER_LIMIT = 85;//temp values, find the real ones in the CAD
    private static final int FOURBAR_LOWER_LIMIT = -40;
    private static final double MANUAL_DEADZONE = 0.3;

    private static final double BOTTOM_SETPOINT = 0;
    private static final double MID_SETPOINT = 35;
    private static final double TOP_SETPOINT = 70;

    //arbitrarily set, input the real values once we know them
    private static final int MOTOR_ID = 2;

    private CANSparkMax fourbarMotor;
    private RelativeEncoder relativeEncoder;
    private SparkMaxPIDController pidController;

    private boolean menuPressed;
    private boolean rbPressed;
    private double speed;
    private double targetSetpoint;
    private ControlMode controlMode;
    private Setpoint setpoint;

    private static final double P = 0.1;
    private static final double I = 0.0;
    private static final double D = 1;
    private static final double F = 0.0;

    private static Fourbar instance = new Fourbar();

    public Fourbar(){
        fourbarMotor = new CANSparkMax(MOTOR_ID, MotorType.kBrushless);
        relativeEncoder = fourbarMotor.getEncoder();
        pidController = fourbarMotor.getPIDController();

        pidController.setP(P);
        pidController.setI(I);
        pidController.setD(D);
        pidController.setFF(F);

        pidController.setOutputRange(-0.5, 0.5);
    }

    private enum ControlMode {
        MANUAL, PID
    }

    private enum Setpoint {
        BOTTOM, MID, TOP
    }

    public static void fourbarInit() {
        instance.controlMode = ControlMode.MANUAL;
        instance.setpoint = Setpoint.BOTTOM;
        instance.speed = 0;
        instance.targetSetpoint = 0;
        instance.fourbarMotor.setIdleMode(IdleMode.kBrake);
    }

    /**
     * The method that will be run from Robot.java
     */
    public static void run() {
        instance.speed = getSpeed();
        cycleTargetSetpoint();

        if(joystick.getRawButton(7))
            instance.relativeEncoder.setPosition(0);

        if(getSwitchControlMode()) {
            if(instance.controlMode == ControlMode.MANUAL) {
                instance.controlMode = ControlMode.PID;
            } else if(instance.controlMode == ControlMode.PID) {
                instance.controlMode = ControlMode.MANUAL;
            }
        }
        
        if(instance.controlMode == ControlMode.MANUAL) {
            manualControl();
        } else if(instance.controlMode == ControlMode.PID) {
            pidControl();
        }

        SmartDashboard.putNumber("FB Speed", instance.speed);
        SmartDashboard.putString("FB Control Mode", instance.controlMode.toString());
        SmartDashboard.putNumber("FB Position", instance.relativeEncoder.getPosition());
        SmartDashboard.putNumber("FB Target Setpoint", instance.targetSetpoint);
        SmartDashboard.putNumber("FB Motor Power", instance.fourbarMotor.get());
    }

    /**Commands the fourbar position in degrees */
    private static void pidControl(){
        if(instance.setpoint == Setpoint.BOTTOM) {
            instance.targetSetpoint = BOTTOM_SETPOINT;
        } else if(instance.setpoint == Setpoint.MID) {
            instance.targetSetpoint = MID_SETPOINT;
        } else if(instance.setpoint == Setpoint.TOP) {
            instance.targetSetpoint = TOP_SETPOINT;
        }

        instance.pidController.setReference(instance.targetSetpoint, CANSparkMax.ControlType.kPosition);
    }

    private static void manualControl(){
        instance.fourbarMotor.set(-instance.speed);
    }

    private static boolean getSwitchControlMode() {
        if(!instance.menuPressed && joystick.getRawButton(5)) {
            instance.menuPressed = true;
            return true;
        } else if(instance.menuPressed && !joystick.getRawButton(5)) {
            instance.menuPressed = false;
            return false;
        } else return false;
    }

    private static boolean getSwitchSetpoint() {
        if(!instance.rbPressed && joystick.getRawButton(10)) {
            instance.rbPressed = true;
            return true;
        } else if(instance.rbPressed && !joystick.getRawButton(10)) {
            instance.rbPressed = false;
            return false;
        } else return false;
    }

    private static double getSpeed() {
        if(Math.abs(joystick.getRawAxis(5)) >= MANUAL_DEADZONE) {
            return joystick.getRawAxis(5)/3;
        }
        else return 0;
    }
    
    private static void cycleTargetSetpoint() {
        if(getSwitchSetpoint()) {
            if(instance.setpoint == Setpoint.BOTTOM) {
                instance.setpoint = Setpoint.MID;
            } else if(instance.setpoint == Setpoint.MID) {
                instance.setpoint = Setpoint.TOP;
            } else if(instance.setpoint == Setpoint.TOP) {
                instance.setpoint = Setpoint.BOTTOM;
            }
        }
    }
}