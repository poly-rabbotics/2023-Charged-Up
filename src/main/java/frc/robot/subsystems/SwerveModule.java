// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.SmartPrintable;

/** 
 * Class for managing and manipulating a swerve module. 
 */
public class SwerveModule extends SmartPrintable {
    private static final double CONVERSION_FACTOR_ROTATION = Math.toRadians(150 / 7);                         // Rotations to radians.
    private static final double CONVERSION_FACTOR_MOVEMENT = 6.75;                                            // Rotations to meters.
    private static final double CONVERSION_FACTOR_ROTATION_VELOCITY = CONVERSION_FACTOR_ROTATION * (1 / 60);  // RPM to radians per second.
    private static final double CONVERSION_FACTOR_MOVEMENT_VELOCITY = CONVERSION_FACTOR_MOVEMENT * (1 / 60);  // RPM to meters per second.

    private static final double PID_P = 0.5;
    private static final double PID_I = 0.0;
    private static final double PID_D = 0.0;

    private static final double ROCK_PID_P = 0.1;
    private static final double ROCK_PID_I = 0.0;
    private static final double ROCK_PID_D = 0.0;

    private final SwerveModulePosition position;

    private final CANSparkMax rotationMotor;  // The motor responsible for rotating the module.
    private final CANSparkMax movementMotor;  // The motor responsible for creating movement in the module.
    private final CANCoder angularEncoder;    // Cancoder responsible for tracking the angle of the module.

    private final RelativeEncoder rotationEncoder; // Relative encoder for tracking rotational movement.
    private final RelativeEncoder movementEncoder; // Relative encoder for tracking translational movement.

    private final PIDController rotationController;
    private final PIDController rockController;

    private final RelativePosition physicalPosition;
    private final Angle canCoderOffset;
    private final double coefficient;

    // Set to NaN if not in rock mode, NaN does not equal itself by definition
    // (see some IEEE standard or something) and so this is how rock mode is 
    // checked. Max speed works the same way.
    private double rockPos = Double.NaN;
    private double maxSpeed = Double.NaN;

    // Very important changes that fix everything!

    private enum RelativePosition {
        FRONT_RIGHT (  1.0,  1.0 ),
        FRONT_LEFT  ( -1.0,  1.0 ),
        BACK_LEFT   ( -1.0, -1.0 ),
        BACK_RIGHT  (  1.0, -1.0 );

        private boolean front = true;
        private boolean right = true;

        RelativePosition(double x, double y) {
            right = Math.signum(x) > 0.0;
            front = Math.signum(y) > 0.0;
        }

        public String asString() {
            String str = "";
            
            if (front) {
                str += "Front ";
            } else {
                str += "Back ";
            }

            if (right) {
                str += "Right";
            } else {
                str += "Left";
            }

            return str;
        }

        public static RelativePosition fromTranslation(Translation2d translation) {
            var x_sign = Math.signum(translation.getX()) > 0.0;
            var y_sign = Math.signum(translation.getY()) > 0.0;

            if (x_sign && y_sign) {
                return FRONT_RIGHT;
            } else if (x_sign) {
                return BACK_RIGHT;
            } else if (y_sign) {
                return FRONT_LEFT;
            } 
                
            return BACK_LEFT;
        }
    }

    public SwerveModule(
        int movementMotorID, 
        int rotationalMotorID, 
        int canCoderID, 
        Angle canCoderOffset, 
        double coefficient,
        Translation2d physicalPosition
    ) {
        super();
        
        this.physicalPosition = RelativePosition.fromTranslation(physicalPosition);
        this.canCoderOffset = canCoderOffset.clone();
        this.coefficient = coefficient;

        rotationMotor = new CANSparkMax(rotationalMotorID, MotorType.kBrushless);
        rotationMotor.setInverted(false);
        rotationMotor.setSmartCurrentLimit(30);
        
        movementMotor = new CANSparkMax(movementMotorID, MotorType.kBrushless);
        movementMotor.setInverted(false);
        movementMotor.setIdleMode(IdleMode.kBrake);
        movementMotor.setSmartCurrentLimit(40);

        angularEncoder = new CANCoder(canCoderID);
        angularEncoder.configFactoryDefault();

        // Magic and forbidden config from the code orange wizards. Makes the 
        // encoder initialize to absolute.
        angularEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        angularEncoder.configFeedbackCoefficient(
            // Since the default coefficiant used for degrees is not 
            // particularly intuitive we just grab it and run a deg -> rad
            // conversion on it.
            Math.toRadians(angularEncoder.configGetFeedbackCoefficient()), 
            "rad", 
            SensorTimeBase.PerSecond
        );

        rotationEncoder = rotationMotor.getEncoder();
        rotationEncoder.setPosition(angularEncoder.getPosition());
        rotationEncoder.setPositionConversionFactor(CONVERSION_FACTOR_ROTATION);
        rotationEncoder.setVelocityConversionFactor(CONVERSION_FACTOR_ROTATION_VELOCITY);

        movementEncoder = movementMotor.getEncoder();
        movementEncoder.setPosition(0.0);
        movementEncoder.setPositionConversionFactor(CONVERSION_FACTOR_MOVEMENT);
        movementEncoder.setVelocityConversionFactor(CONVERSION_FACTOR_MOVEMENT_VELOCITY);
        
        rotationController = new PIDController(PID_P, PID_I, PID_D);
        rotationController.enableContinuousInput(0.0, Angle.TAU);
        rotationController.setTolerance(0.005); // This is more precise than before, TODO: test.

        rockController = new PIDController(ROCK_PID_P, ROCK_PID_I, ROCK_PID_D);
        rockController.setTolerance(0.5);

        position = new SwerveModulePosition(
            movementEncoder.getPosition() * CONVERSION_FACTOR_MOVEMENT, 
            new Rotation2d(angularEncoder.getPosition())
        );
    }

    /**
     * Sets the desired module state for this module. This must be run 
     * repeatedly to continue PID calculations.
     */
    public void setDesiredState(SwerveModuleState state) {
        double currentPosition = (angularEncoder.getPosition() + canCoderOffset.radians()) % Angle.TAU;
        state = SwerveModuleState.optimize(state, new Rotation2d(currentPosition));
        
        if (rockPos != rockPos) {
            movementMotor.set(
                Math.abs(state.speedMetersPerSecond) > maxSpeed 
                    ? Math.signum(state.speedMetersPerSecond) * maxSpeed 
                    : state.speedMetersPerSecond
            );
        } else {
            movementMotor.set(rockController.calculate(getDistanceTraveled(), rockPos));
        }

        double calculation = rotationController.calculate(currentPosition, (state.angle.getRadians() + Angle.TAU) % Angle.TAU);
        rotationMotor.set(calculation * coefficient);

        position.angle = new Rotation2d(angularEncoder.getPosition());
        position.distanceMeters = movementEncoder.getPosition() * CONVERSION_FACTOR_MOVEMENT;
    }

    /**
     * True if the module should be in rock mode.
     */
    public void setRockMode(boolean shouldHold) {
        if (!shouldHold) {
            rockPos = Double.NaN;
            return;
        } else if (rockPos != rockPos) {
            rockPos = getDistanceTraveled();
        }
    }

    /**
     * True if in rock mode.
     */
    public boolean inRockMode() {
        return rockPos == rockPos;
    }

    /**
     * Sets this module's maximum movement speed. Use NaN for no limit.
     */
    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = maxSpeed;
    }

    /**
     * Gets this module's maximum movement speed
     */
    public double getMaxSpeed() {
        return maxSpeed;
    }

    /**
     * Gets distance traveled, should not be used for ablsolute distances as 
     * this function currently makes no guarantee as to the starting position
     * of the module. (This can be mitigated if you zero positions, but it will
     * interupt odometry).
     */
    public double getDistanceTraveled() {
        return movementEncoder.getPosition();
    }

    @Override
    public void print() {
        SmartDashboard.putNumber("Module " + physicalPosition.asString() + " Position", angularEncoder.getPosition());
        SmartDashboard.putNumber("Module " + physicalPosition.asString() + " Position mod tau", angularEncoder.getPosition() % Angle.TAU);
        SmartDashboard.putNumber("Module " + physicalPosition.asString() + " Position + off", angularEncoder.getPosition() + canCoderOffset.radians());
        SmartDashboard.putNumber("Module " + physicalPosition.asString() + " Position + off mod tau", (angularEncoder.getPosition() + canCoderOffset.radians()) % Angle.TAU);
        SmartDashboard.putNumber("Module " + physicalPosition.asString() + " Position (Distance) ", movementEncoder.getPosition());
    }

    /**
     * Gets the angle of the module. 
     */
    public SwerveModulePosition getPosition() {
        return position;
    }

    /**
     * Sets movement position to zero, will mess up odometry.
     */
    public void zeroPositions() {
        movementEncoder.setPosition(0.0);
    }
}
