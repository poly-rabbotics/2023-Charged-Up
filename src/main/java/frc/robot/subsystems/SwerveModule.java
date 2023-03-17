// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** 
 * Class for managing and manipulating a swerve module. 
 */
public class SwerveModule {
    private static final double CONVERSION_FACTOR_ROTATION = 150 / 7;                                         // Rotations to degrees.
    private static final double CONVERSION_FACTOR_MOVEMENT = 6.75;                                            // Rotations to meters.
    private static final double CONVERSION_FACTOR_ROTATION_VELOCITY = CONVERSION_FACTOR_ROTATION * (1 / 60);  // RPM to degrees per second.
    private static final double CONVERSION_FACTOR_MOVEMENT_VELOCITY = CONVERSION_FACTOR_MOVEMENT * (1 / 60);  // RPM to meters per second.

    private static double PID_P = 0.005301;
    private static double PID_I = 0;
    private static double PID_D = 0;

    private final SwerveModulePosition position;

    private final CANSparkMax rotationMotor;  // The motor responsible for rotating the module.
    private final CANSparkMax movementMotor;  // The motor responsible for creating movement in the module.
    private final CANCoder angularEncoder;    // Cancoder responsible for tracking the angle of the module.

    private final RelativeEncoder rotationEncoder; // Relative encoder for tracking rotational movement.
    private final RelativeEncoder movementEncoder; // Relative encoder for tracking translational movement.

    private final PIDController rotationController;

    private final double canCoderOffset;
    private final double coefficient;

    public SwerveModule(int movementMotorID, int rotationalMotorID, int canCoderID, double canCoderOffset, double coefficient) {
        this.canCoderOffset = canCoderOffset;
        this.coefficient = coefficient;

        rotationMotor = new CANSparkMax(rotationalMotorID, MotorType.kBrushless);
        rotationMotor.setInverted(false);
        //rotationMotor.setSmartCurrentLimit(40);
        
        movementMotor = new CANSparkMax(movementMotorID, MotorType.kBrushless);
        movementMotor.setInverted(false);
        movementMotor.setIdleMode(IdleMode.kBrake);
        movementMotor.setSmartCurrentLimit(40);

        angularEncoder = new CANCoder(canCoderID);
        angularEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition, 1000);

        rotationEncoder = rotationMotor.getEncoder();
        rotationEncoder.setPosition(angularEncoder.getPosition());
        rotationEncoder.setPositionConversionFactor(CONVERSION_FACTOR_ROTATION);
        rotationEncoder.setVelocityConversionFactor(CONVERSION_FACTOR_ROTATION_VELOCITY);

        movementEncoder = movementMotor.getEncoder();
        movementEncoder.setPosition(0.0);
        movementEncoder.setPositionConversionFactor(CONVERSION_FACTOR_MOVEMENT);
        movementEncoder.setVelocityConversionFactor(CONVERSION_FACTOR_MOVEMENT_VELOCITY);
        
        rotationController = new PIDController(PID_P, PID_I, PID_D);
        rotationController.enableContinuousInput(0.0, 360.0);
        rotationController.setTolerance(0.5);

        position = new SwerveModulePosition(movementEncoder.getPosition() * CONVERSION_FACTOR_MOVEMENT, new Rotation2d(angularEncoder.getPosition() / 180.0 * Math.PI));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            movementMotor.set(0.0);
            rotationMotor.set(0.0);
            return;
        }
        
        double currentPosition = (angularEncoder.getPosition() + canCoderOffset) % 360.0;
        state = SwerveModuleState.optimize(state, new Rotation2d(currentPosition * Math.PI / 180.0));
        double calculation = rotationController.calculate(currentPosition, (state.angle.getDegrees() + 360.0) % 360.0);
        
        movementMotor.set(state.speedMetersPerSecond);
        rotationMotor.set(calculation * coefficient);

        position.angle = new Rotation2d(angularEncoder.getPosition());
        position.distanceMeters = movementEncoder.getPosition() * CONVERSION_FACTOR_MOVEMENT;
    }

    public void print() {
        SmartDashboard.putNumber("Module " + angularEncoder.getDeviceID() + " Position", angularEncoder.getPosition());
        SmartDashboard.putNumber("Module " + angularEncoder.getDeviceID() + " Position mod 360", angularEncoder.getPosition() % 360);
        SmartDashboard.putNumber("Module " + angularEncoder.getDeviceID() + " Position + off", angularEncoder.getPosition() + canCoderOffset);
        SmartDashboard.putNumber("Module " + angularEncoder.getDeviceID() + " Position + off mod 360", (angularEncoder.getPosition() + canCoderOffset) % 360);
    }

    public SwerveModulePosition getPosition() {
        return position;
    }
}
