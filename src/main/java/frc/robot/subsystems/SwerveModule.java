// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.systems.SwerveDrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class SwerveModule {
    private static double PID_P = 0.0000001;
    private static double PID_I = 0;
    private static double PID_D = 0;

    private CANSparkMax rotationalMotor;
    private CANSparkMax movementMotor;
    private CANCoder rotationalEncoder;
    private PIDController controller;
    private double CANCoderOffset; //in degrees

    private class DirectionSet {
        public double movement;
        public boolean invert;

        public DirectionSet(double movement, boolean invert) {
            this.movement = movement;
            this.invert = invert;
        }
    }

    public SwerveModule(int movementMotorID, int rotationalMotorID, int canCoderID, double CANCoderOffset) {
        rotationalMotor = new CANSparkMax(rotationalMotorID, MotorType.kBrushless);
        movementMotor = new CANSparkMax(movementMotorID, MotorType.kBrushless);
        rotationalEncoder = new CANCoder(canCoderID);
        controller = new PIDController(PID_P, PID_I, PID_D);
        this.CANCoderOffset = CANCoderOffset;
    }

    private DirectionSet convertAngleToDirection(double setpoint1) {
        double currentPosition = rotationalEncoder.getPosition() - CANCoderOffset;
        
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
        double calculation = controller.calculate(rotationalEncoder.getPosition(), directionSet.movement);
        
        if (directionSet.invert)
            rotationalMotor.setInverted(!rotationalMotor.getInverted());
        
        rotationalMotor.set(calculation);
        movementMotor.set(speed);
        SmartDashboard.putNumber("Module " + rotationalEncoder.getDeviceID() + " CANCoder", rotationalEncoder.getPosition() % 360);
    }
    //for making PID adjustments faster rather than changing the code and re-deploying
    public void adjustPIDs() {
        SmartDashboard.putNumber("P", PID_P);
        SmartDashboard.putNumber("I", PID_I);
        SmartDashboard.putNumber("D", PID_D);

        if (SwerveDrive.testController.getRawButtonPressed(6)) {
            PID_P += 0.00000001;
        } 
        else if (SwerveDrive.testController.getRawButtonPressed(5)) {
            PID_P -= 0.00000001;
        }
        if (SwerveDrive.testController.getRawButtonPressed(1)) {
            PID_I += 0.00000001;
        } 
        else if (SwerveDrive.testController.getRawButtonPressed(4)) {
            PID_I -= 0.00000001;
        } 
        if (SwerveDrive.testController.getPOV() == 180) {
            PID_D += 0.00000001;
        } 
        else if (SwerveDrive.testController.getPOV() == 0) {
            PID_D -= 0.00000001;
        }
    }
}
