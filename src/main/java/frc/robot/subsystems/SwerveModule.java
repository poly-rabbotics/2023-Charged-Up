// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
    private static final double CAN_SPARK_MAX_RATED_AMPS = 60.0;

    private static final double PID_P = 0.5;
    private static final double PID_I = 0.0;
    private static final double PID_D = 0.0;

    private static final double ROCK_PID_P = 0.05;
    private static final double ROCK_PID_I = 0.0;
    private static final double ROCK_PID_D = 0.0;

    private final SwerveModulePosition position;
    private final SwerveModuleRunner moduleRunner;
    private final ScheduledExecutorService runnerThread;

    private final CANSparkMax rotationMotor;  // The motor responsible for rotating the module.
    private final CANSparkMax movementMotor;  // The motor responsible for creating movement in the module.
    private final CANCoder angularEncoder;    // Cancoder responsible for tracking the angle of the module.

    private final RelativeEncoder rotationEncoder; // Relative encoder for tracking rotational movement.
    private final RelativeEncoder movementEncoder; // Relative encoder for tracking translational movement.

    private final PIDController rotationController;
    private final PIDController rockController;

    private final RelativePosition physicalPosition;
    private final Angle canCoderOffset;

    private SlewRateLimiter accelerationLimit;
    private WriteLock<SwerveModuleState> desiredState;

    // Set to NaN if not in rock mode, NaN does not equal itself by definition
    // (see some IEEE standard or something) and so this is how rock mode is 
    // checked. Max speed works the same way.
    private double rockPos = Double.NaN;
    private double maxSpeed = Double.NaN;
    private double accelerationRate = Double.NaN;

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

    private class SwerveModuleRunner implements Runnable {
        private SwerveModule parentModule;

        public SwerveModuleRunner(SwerveModule parentModule) {
            this.parentModule = parentModule;
        }
        
        @Override
        public void run() {
            double currentPosition = (angularEncoder.getPosition() + canCoderOffset.radians()) % Angle.TAU;

            // Lock the parent module's desired state for reading.
            SwerveModuleState parentState = parentModule.desiredState.lock();
            SwerveModuleState state = SwerveModuleState.optimize(parentState, new Rotation2d(currentPosition));

            // Unlock to allow writing again. NOTE: the `parentState` variable
            // MAY NOT BE USED beyond this point.
            parentModule.desiredState.unlock();

            /*
             * There are two important reasons to run `SwerveModuleState.optomize()`. The first is obvious, make swerve
             * more efficient. The other is to make a value-wise copy of the module state to avoid race conditions, as the 
             * method creates a new `SwerveModuleState` each time. See this link for source:
             * https://github.wpilib.org/allwpilib/docs/release/java/src-html/edu/wpi/first/math/kinematics/SwerveModuleState.html#line.74
             */
        
            if (rockPos != rockPos) {
                var desiredSpeed = Math.abs(state.speedMetersPerSecond) > maxSpeed 
                    ? Math.signum(state.speedMetersPerSecond) * maxSpeed 
                    : state.speedMetersPerSecond;
                movementMotor.set(
                    accelerationRate == accelerationRate
                        ? accelerationLimit.calculate(desiredSpeed)
                        : desiredSpeed
                );
            } else {
                movementMotor.set(rockController.calculate(getDistanceTraveled(), rockPos));
            }

            double calculation = rotationController.calculate(currentPosition, (state.angle.getRadians() + Angle.TAU) % Angle.TAU);
            rotationMotor.set(calculation);

            position.angle = new Rotation2d(angularEncoder.getPosition());
            position.distanceMeters = movementEncoder.getPosition() * CONVERSION_FACTOR_MOVEMENT;
        }
    }

    public SwerveModule(
        int movementMotorID, 
        int rotationalMotorID, 
        int canCoderID, 
        Angle canCoderOffset, 
        Translation2d physicalPosition
    ) {
        super();
        
        this.physicalPosition = RelativePosition.fromTranslation(physicalPosition);
        this.canCoderOffset = canCoderOffset.clone();

        rotationMotor = new CANSparkMax(rotationalMotorID, MotorType.kBrushless);
        rotationMotor.setInverted(true);
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
        rotationController.setTolerance(0.01);

        rockController = new PIDController(ROCK_PID_P, ROCK_PID_I, ROCK_PID_D);
        rockController.setTolerance(1);

        position = new SwerveModulePosition(
            movementEncoder.getPosition() * CONVERSION_FACTOR_MOVEMENT, 
            new Rotation2d(angularEncoder.getPosition())
        );

        accelerationLimit = new SlewRateLimiter(1.5);
        desiredState = new WriteLock<>(new SwerveModuleState());
        moduleRunner = new SwerveModuleRunner(this);

        runnerThread = Executors.newSingleThreadScheduledExecutor();
        runnerThread.scheduleAtFixedRate(moduleRunner, 0, 20, TimeUnit.MILLISECONDS);
    }

    /**
     * Sets the desired module state for this module. This must be run 
     * repeatedly to continue PID calculations.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = this.desiredState.lock();
        state = desiredState;
        this.desiredState.unlock(state);
    }

    /**
     * True if the module should be in rock mode.
     */
    public void setRockMode(boolean shouldHold) {
        if (!shouldHold) {
            rockPos = Double.NaN;
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
     * Sets whether or not to limit the acceleration of the module.
     */
    public void setAccelerationRate(double accelerationRate) {
        if (accelerationRate != this.accelerationRate && accelerationRate == accelerationRate) {
            accelerationLimit = new SlewRateLimiter(accelerationRate);
        }

        this.accelerationRate = accelerationRate;
    }

    /**
     * Gets whether or not the drive is limiting its acceleration.
     */
    public double getAccelerationRate() {
        return accelerationRate;
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

    /**
     * Gets the reported tempurature of the rotation motor in celsius.
     */
    public double getRotationMotorTemp() {
        return rotationMotor.getMotorTemperature();
    }

    /**
     * Gets the reported tempurature of the movement motor in celsius.
     */
    public double getMovementMotorTemp() {
        return movementMotor.getMotorTemperature();
    }

    /**
     * Gets the power being outputted by the rotation motor's controller in amps.
     */
    public double getRotationMotorCurrent() {
        return rotationMotor.getOutputCurrent();
    }

    /**
     * Gets the power being outputted by the movement motor's controller in amps.
     */
    public double getMovementMotorCurrent() {
        return movementMotor.getOutputCurrent();
    }

    /**
     * Gets the sum of all motor's current in amps.
     */
    public double getAppliedCurrent() {
        return getRotationMotorCurrent() + getMovementMotorCurrent();
    }

    /**
     * Gets the percentage of the maximum rated amperage of the motor 
     * controllers currently being hit by the module.
     */
    public double getPercentRatedCurrent() {
        return getAppliedCurrent() / (2.0 * CAN_SPARK_MAX_RATED_AMPS);
    }

    @Override
    public void print() {
        SmartDashboard.putNumber("Module " + physicalPosition.asString() + "(ids: " + movementMotor.getDeviceId() + ", " + rotationMotor.getDeviceId() + ", " + angularEncoder.getDeviceID() + ") Position", Math.toDegrees(angularEncoder.getPosition()));
        SmartDashboard.putNumber("Module " + physicalPosition.asString() + "(ids: " + movementMotor.getDeviceId() + ", " + rotationMotor.getDeviceId() + ", " + angularEncoder.getDeviceID() + ") Position mod 360", Math.toDegrees(angularEncoder.getPosition() % Angle.TAU));
        SmartDashboard.putNumber("Module " + physicalPosition.asString() + "(ids: " + movementMotor.getDeviceId() + ", " + rotationMotor.getDeviceId() + ", " + angularEncoder.getDeviceID() + ") Position + off", Math.toDegrees(angularEncoder.getPosition() + canCoderOffset.radians()));
        SmartDashboard.putNumber("Module " + physicalPosition.asString() + "(ids: " + movementMotor.getDeviceId() + ", " + rotationMotor.getDeviceId() + ", " + angularEncoder.getDeviceID() + ") Position + off mod 360", Math.toDegrees((angularEncoder.getPosition() + canCoderOffset.radians()) % Angle.TAU));
        SmartDashboard.putNumber("Module " + physicalPosition.asString() + "(ids: " + movementMotor.getDeviceId() + ", " + rotationMotor.getDeviceId() + ", " + angularEncoder.getDeviceID() + ") Position (Distance) ", movementEncoder.getPosition());
        SmartDashboard.putNumber("Module " + physicalPosition.asString() + "(ids: " + movementMotor.getDeviceId() + ", " + rotationMotor.getDeviceId() + ", " + angularEncoder.getDeviceID() + ") Movement Speed", movementMotor.get());
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
