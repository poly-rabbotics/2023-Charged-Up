// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;

/** Add your docs here. */
public class SwerveModule {
    private static final double PID_P = 0.0000001;
    private static final double PID_I = 0;
    private static final double PID_D = 0;

    private CANSparkMax rotationalMotor;
    private CANSparkMax movementMotor;
    private CANCoder rotationalEncoder;
    private PIDController controller;

    private class DirectionSet {
        public double movement;
        public boolean invert;

        public DirectionSet(double movement, boolean invert) {
            this.movement = movement;
            this.invert = invert;
        }
    }

    public SwerveModule(int rotationalMotorID, int movementMotorID, int canCoderID) {
        rotationalMotor = new CANSparkMax(rotationalMotorID, MotorType.kBrushless);
        movementMotor = new CANSparkMax(movementMotorID, MotorType.kBrushless);
        rotationalEncoder = new CANCoder(canCoderID);
        controller = new PIDController(PID_P, PID_I, PID_D);
    }

    private DirectionSet convertAngleToDirection(double setpoint1) {
        double currentPosition = rotationalEncoder.getPosition();
        
        // We're going to start finding the quickest way to get to either the
        // given point, or 180 degrees from it. In the case of the latter we
        // will invert the motor.

        double setpoint2 = (setpoint1 + 180.0);

        double movement1 = setpoint1 - currentPosition;
        double movement2 = 360.0 + movement1;
        double movement3 = setpoint2 - currentPosition;
        double movement4 = 360.0 - movement3;

        double minimumMovement = Math.min(Math.min(movement1, movement2), Math.min(movement3, movement4));
        boolean inversionRequired = (minimumMovement == movement3) || (minimumMovement == movement4);

        return new DirectionSet(minimumMovement, inversionRequired);
    }

    /**
     * Applies the given movement vector to the swerve module.
     * @param rotation Rotation in degrees.
     * @param speed Native motor speed, 1.0 to -1.0.
     */
    public void setMovementVector(double rotation, double speed) {
        controller.setSetpoint(rotation);
        controller.reset();

        DirectionSet directionSet = convertAngleToDirection(rotation);
        double calculation = controller.calculate(rotationalEncoder.getPosition() + directionSet.movement);
        
        if (directionSet.invert)
            rotationalMotor.setInverted(!rotationalMotor.getInverted());

        rotationalMotor.set(calculation);
        movementMotor.set(speed);
    }
}
