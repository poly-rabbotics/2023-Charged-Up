// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
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

    private static double MOVEMENT_PID_P = 0.0005301;
    private static double MOVEMENT_PID_I = 0;
    private static double MOVEMENT_PID_D = 0;

    private final CANSparkMax rotationMotor;  // The motor responsible for rotating the module.
    private final CANSparkMax movementMotor;  // The motor responsible for creating movement in the module.
    private final CANCoder angularEncoder;    // Cancoder responsible for tracking the angle of the module.

    private final RelativeEncoder rotationEncoder; // Relative encoder for tracking rotational movement.
    private final RelativeEncoder movementEncoder; // Relative encoder for tracking translational movement.

    private final PIDController rotationController;
    private final PIDController movementController;
    private final double canCoderOffset;
    private final double coefficient;

    private double autoMovementSetpoint = 0.0;
    private double autoRotationSetpoint = 0.0;

    public SwerveModule(int movementMotorID, int rotationalMotorID, int canCoderID, double canCoderOffset, double coefficient) {
        this.canCoderOffset = canCoderOffset;
        this.coefficient = coefficient;

        rotationMotor = new CANSparkMax(rotationalMotorID, MotorType.kBrushless);
        rotationMotor.setInverted(false);
        
        movementMotor = new CANSparkMax(movementMotorID, MotorType.kBrushless);
        movementMotor.setInverted(true);
        movementMotor.setIdleMode(IdleMode.kBrake);

        angularEncoder = new CANCoder(canCoderID);

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

        movementController = new PIDController(MOVEMENT_PID_P, MOVEMENT_PID_I, MOVEMENT_PID_D);
        movementController.setTolerance(0.5);
    }

    /**
     * Applies the given movement vector to the swerve module.
     * @param speed Speed in native motor speed units, -1.0 to 1.0.
     * @param rotation Rotation in degrees.
     */
    public void setMovementVector(double rotation, double speed) {
        double currentPosition = (angularEncoder.getPosition() + canCoderOffset) % 360.0;
        double calculation = rotationController.calculate(currentPosition, rotation % 360.0);
        
        SmartDashboard.putNumber("Module " + angularEncoder.getDeviceID() + " Calculation", calculation);
        SmartDashboard.putNumber("Module " + angularEncoder.getDeviceID() + " Rotation", rotation);
        SmartDashboard.putNumber("Module " + angularEncoder.getDeviceID() + " Position", angularEncoder.getPosition());
        SmartDashboard.putNumber("Module " + angularEncoder.getDeviceID() + " Position mod 360", angularEncoder.getPosition() % 360);

        rotationMotor.set(calculation * coefficient);
        movementMotor.set(speed);
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(movementEncoder.getVelocity(), new Rotation2d(angularEncoder.getPosition()));
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
        
        movementMotor.set(state.speedMetersPerSecond / 5);
        rotationMotor.set(calculation * coefficient);

        SmartDashboard.putNumber("Module " + angularEncoder.getDeviceID() + " Calculation", calculation);
        SmartDashboard.putNumber("Module " + angularEncoder.getDeviceID() + " Rotation", (state.angle.getDegrees() + 360) % 360.0);
        SmartDashboard.putNumber("Module " + angularEncoder.getDeviceID() + " Position", angularEncoder.getPosition());
        SmartDashboard.putNumber("Module " + angularEncoder.getDeviceID() + " Position mod 360", angularEncoder.getPosition() % 360);
        //setMovementVector(0.0, (state.angle.getDegrees() + 360) % 360);
    }

    public void zeroCANCoder() {
        angularEncoder.setPosition(0.0);
    }

    /**
     * Sets the setpoints used in run() for auto.
     * @param distance Distance from the current point to move, in meters.
     * @param rotation Absolute rotation, direction in which to move.
     */
    public void setAutoSetpoints(double distance, double rotation) {
        double distanceInRotations = distance / CONVERSION_FACTOR_MOVEMENT;
        double currentRotations = movementEncoder.getPosition();

        autoMovementSetpoint = currentRotations + distanceInRotations;
        autoRotationSetpoint = rotation;
    }

    /**
     * Runs the module's auto using previously given setpoints. Returns true if
     * the setpoints have been reached.
     */
    public boolean autoRun() {
        double movementCalculation = movementController.calculate(movementEncoder.getPosition(), autoMovementSetpoint);
        double rotationCalculation = rotationController.calculate((angularEncoder.getPosition() + canCoderOffset) % 360.0, autoRotationSetpoint);

        rotationMotor.set(rotationCalculation);

        if (rotationCalculation != 0.0) {
            return false;
        }

        movementMotor.set(movementCalculation);

        if (movementCalculation == 0.0) {
            return true;
        }

        return false;
    }
}
