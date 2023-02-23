// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class SwerveModule {
    private static double PID_P = 0.000001;
    private static double PID_I = 0;
    private static double PID_D = 0;

    private CANSparkMax rotationalMotor;
    private CANSparkMax movementMotor;
    private CANCoder rotationalEncoder;
    private PIDController controller;
    private double canCoderOffset;
    private double coefficient;

    public SwerveModule(int movementMotorID, int rotationalMotorID, int canCoderID, double canCoderOffset, double coefficient) {
        rotationalMotor = new CANSparkMax(rotationalMotorID, MotorType.kBrushless);
        rotationalMotor.setInverted(false);
        
        movementMotor = new CANSparkMax(movementMotorID, MotorType.kBrushless);
        movementMotor.setInverted(false);
        movementMotor.setIdleMode(IdleMode.kBrake);

        rotationalEncoder = new CANCoder(canCoderID);
        controller = new PIDController(PID_P, PID_I, PID_D);
        controller.enableContinuousInput(0.0, 360.0);
        this.canCoderOffset = canCoderOffset;

        controller.setTolerance(0.5); // 0.5 degrees, maybe change this 
        rotationalEncoder.setPosition(0.0);

        this.coefficient = coefficient;
    }

    /**
     * Applies the given movement vector to the swerve module.
     * @param speed Desired velocity in meters per second.
     * @param rotation Rotation in degrees.
     */
    public void setMovementVector(double rotation, double speed) {
        double currentPosition = (rotationalEncoder.getPosition() + canCoderOffset) % 360.0;
        double calculation = controller.calculate(currentPosition, rotation % 360.0);
        
        SmartDashboard.putNumber("Module " + rotationalEncoder.getDeviceID() + " Calculation", calculation);
        SmartDashboard.putNumber("Module " + rotationalEncoder.getDeviceID() + " Rotation", rotation);
        SmartDashboard.putNumber("Module " + rotationalEncoder.getDeviceID() + " Position", rotationalEncoder.getPosition());
        SmartDashboard.putNumber("Module " + rotationalEncoder.getDeviceID() + " Position mod 360", rotationalEncoder.getPosition() % 360);

        rotationalMotor.set(calculation * coefficient);
        movementMotor.set(speed);
    }

    public void addToP(double increment) {
        PID_P += increment;

        if (PID_P < 0.0)
            PID_P = 0.0;

        SmartDashboard.putNumber("Swerve " + rotationalEncoder.getDeviceID() + "  P", PID_P);

        controller.setP(PID_P);
    }
    public void addToI(double increment) {
        PID_I += increment;

        if (PID_I < 0.0)
            PID_I = 0.0;

        SmartDashboard.putNumber("Swerve " + rotationalEncoder.getDeviceID() + "  I", PID_I);

        controller.setI(PID_I);
    }
    public void addToD(double increment) {
        PID_D += increment;
        
        if (PID_D < 0.0)
            PID_D = 0.0;
        
        SmartDashboard.putNumber("Swerve " + rotationalEncoder.getDeviceID() + "  D", PID_D);

        controller.setD(PID_D);
    }
    public void resetPos() {
        rotationalEncoder.setPosition(0.0);
    }
}
