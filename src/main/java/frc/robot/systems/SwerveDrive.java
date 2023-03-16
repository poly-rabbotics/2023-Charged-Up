// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems;

import java.util.function.BiFunction;
import java.util.function.Function;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.SwerveMode;
import frc.robot.subsystems.SwerveModule;

/*
 * Manages the swerve drive train.
 */
public class SwerveDrive {
    private static final int MODULE_MOVEMENT_CAN_IDS[] = { 1,  2,   3,   4  };
    private static final int MODULE_ROTATION_CAN_IDS[] = { 5,  6,   7,   8  };
    private static final int MODULE_CANCODER_CAN_IDS[] = { 9,  10,  11,  12 };
    
    private static final double MODULE_CANCODER_OFFSETS[] = { -251.6307 + 90.0, -44.1210 + 90.0, -192.0409 + 90.0, -175.1659 + 90.0 };
    private static final double MODULE_COEFFIENTS[] = { -1.0, -1.0, -1.0, -1.0 };
    
    private static final double LOW_SENSITIVITY_RATIO = 0.02;
    private static final double CURVE_EXPONENT_DIRECTIONAL = 3.0;
    private static final double CURVE_EXPONENT_TURN = 3.0;

    private static final double CHASSIS_SIDE_LENGTH = 0.6;
    private static final double RADIAN_DEGREE_RATIO = Math.PI / 180.0;

    private static final BiFunction<Double, Double, Double> DIRECTION_CURVE = (Double directionThis, Double directionOther) -> {
        double distance = Math.sqrt(directionThis*directionThis + directionOther*directionOther);

        if (distance < 0.1) {
            return 0.0;
        }

        double curvedDistance = Math.pow(distance, CURVE_EXPONENT_DIRECTIONAL);
        double distanceRatio = curvedDistance * distance;

        return directionThis * distanceRatio;
    };

    private static final Function<Double, Double> TURN_CURVE = (Double turn) -> {
        if (Math.abs(turn) < 0.1) {
            return 0.0;
        }

        return Math.pow(turn, CURVE_EXPONENT_TURN);
    };

    private static final SwerveDrive instance = new SwerveDrive();

    private final SwerveModule modules[] = new SwerveModule[4];
    private final SwerveDriveKinematics kinematics;
    private SwerveMode mode = SwerveMode.Headless;

    private SwerveDrive() {
        for (int i = 0; i < MODULE_MOVEMENT_CAN_IDS.length; i++) {
            modules[i] = new SwerveModule(
                MODULE_MOVEMENT_CAN_IDS[i], 
                MODULE_ROTATION_CAN_IDS[i], 
                MODULE_CANCODER_CAN_IDS[i], 
                MODULE_CANCODER_OFFSETS[i], 
                MODULE_COEFFIENTS[i]
            );
        }

        kinematics = new SwerveDriveKinematics(
            new Translation2d(   CHASSIS_SIDE_LENGTH / 2,   CHASSIS_SIDE_LENGTH / 2),
            new Translation2d(  -CHASSIS_SIDE_LENGTH / 2,   CHASSIS_SIDE_LENGTH / 2),
            new Translation2d(  -CHASSIS_SIDE_LENGTH / 2,  -CHASSIS_SIDE_LENGTH / 2),
            new Translation2d(   CHASSIS_SIDE_LENGTH / 2,  -CHASSIS_SIDE_LENGTH / 2)
        );
    }

    /**
     * Sets the current mode of the swerve drive. This changes the behavior of
     * setMovementVector.
     * @param mode The mode in which to operate.
     */
    public static void setMode(SwerveMode mode) {
        instance.mode = mode;
    }

    /**
     * Get the current swerve mode.
     */
    public static SwerveMode getMode() {
        return instance.mode;
    }

    /**
     * Moves all swerve modules so that the face forward relative to the 
     * robot's front.
     */
    public static void resetOrientation() {
        for (SwerveModule module : instance.modules) {
            module.setMovementVector(0.0, 0.0);
        }
    }

    /**
     * Runs swerve, behavior changes based on the drive's mode. Derives speed
     * from directional inputs.
     * @param directionalX The X axis of the directional control, between 1 and -1
     * @param directionalY The Y axis of the directional control, between 1 and -1.
     * @param turn A value between 1 and -1 that determines the turning angle.
     * @param lowSense The angle to move in low sensitivity in degrees, -1 for no movement.
     */
    public static void run(double directionalX, double directionalY, double turn, int lowSense) {
        if (lowSense != -1) {
            double angle = (2 * Math.PI) - ((double)lowSense * RADIAN_DEGREE_RATIO);

            // inverted since the drive is rotated to compensate for joystick stuff
            directionalX = -(Math.sin(angle) * LOW_SENSITIVITY_RATIO);
            directionalY = -(Math.cos(angle) * LOW_SENSITIVITY_RATIO);
            
            runUncurved(directionalX, directionalY, TURN_CURVE.apply(turn));
            return;
        }

        directionalX = DIRECTION_CURVE.apply(directionalX, directionalY);
        directionalY = DIRECTION_CURVE.apply(directionalY, directionalX);
        turn = TURN_CURVE.apply(turn);

        ChassisSpeeds chassisSpeeds;

        if (instance.mode == SwerveMode.Headless) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(directionalX, -directionalY, turn, new Rotation2d(Pigeon.getFeildRelativeRotation() * RADIAN_DEGREE_RATIO));
        } else {
            chassisSpeeds = new ChassisSpeeds(directionalX, -directionalY, turn);
        }

        SwerveModuleState[] moduleStates = instance.kinematics.toSwerveModuleStates(chassisSpeeds);

        for (int i = 0; i < instance.modules.length; i++) {
            instance.modules[i].setDesiredState(moduleStates[i]);
        }
    }

    /**
     * Run the swerve drive exactly by the arguments passed, no cuving will 
     * occure on the given inputs, nor will the be deadbanded.
     * @param directionalX Speed along the x axis. -1.0 - 1.0
     * @param directionalY Speed along the y axis. -1.0 - 1.0
     * @param turn Rate of turn. -1.0 - 1.0
     */
    public static void runUncurved(double directionalX, double directionalY, double turn) {
        ChassisSpeeds chassisSpeeds;

        if (instance.mode == SwerveMode.Headless) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(directionalX, -directionalY, turn, new Rotation2d(Pigeon.getFeildRelativeRotation() * RADIAN_DEGREE_RATIO));
        } else {
            chassisSpeeds = new ChassisSpeeds(directionalX, -directionalY, turn);
        }

        SwerveModuleState[] moduleStates = instance.kinematics.toSwerveModuleStates(chassisSpeeds);

        for (int i = 0; i < instance.modules.length; i++) {
            instance.modules[i].setDesiredState(moduleStates[i]);
        }
    }

    /**
     * Sets the auto setpoints.
     * @param distance Distance from "here" in meters.
     * @param angle Absolute angle of movement.
     */
    public static void autoSetSetpoints(double distance, double angle) {
        for (SwerveModule module : instance.modules) {
            module.setAutoSetpoints(distance, angle);
        }
    }

    /**
     * Runs the swerve modules with previously given setpoints.
     * @return Whether or not the action has completed.
     */
    public static boolean autoRun() {
        boolean completed = true;

        for (SwerveModule module : instance.modules) {
            completed &= module.autoRun();
        }

        return completed;
    }

    private double startTimeBalance = -1.0;
    private Timer balanceTimer = new Timer();

    public static boolean autoBalance() {
        final double TOLERANCE = 12.0;
        final double SPEED = 0.6;

        /* if (Pigeon.getPitch() > TOLERANCE) {
            run(0.0, SPEED, 0.0);
        } else if (Pigeon.getPitch() < -TOLERANCE) {
            run(0.0, -SPEED, 0.0);
        } */

        if (Math.abs(Pigeon.getPitch()) > TOLERANCE) {
            if (instance.startTimeBalance == -1.0) {
                instance.balanceTimer.reset();
                instance.balanceTimer.start();
                instance.startTimeBalance = instance.balanceTimer.get();
                return false;
            }

            if (instance.startTimeBalance < 0.15) {
                return false;
            }

            run(0.0, Math.signum(Pigeon.getPitch()) * SPEED, 0.0, -1);
            return true;
        }

        instance.balanceTimer.stop();
        instance.balanceTimer.reset();
        instance.startTimeBalance = -1.0;
        return false;
    }

    public static void zeroEncoders() {
        for (SwerveModule module : instance.modules) {
            module.zeroCANCoder();
        }
    }

    public static void print() {
        for (SwerveModule module : instance.modules) {
            module.print();
        }
    }
}
