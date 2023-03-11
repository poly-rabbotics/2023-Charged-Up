// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems;

import com.fasterxml.jackson.annotation.JsonCreator.Mode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.SwerveMode;
import frc.robot.subsystems.SwerveModule;

/*
 * Manages the swerve drive train.
 */
public class SwerveDrive {
    /*
     * IDs in order:
     * Upper right
     * Upper left
     * Back left
     * Back right - 
     */

    private static final int MODULE_MOVEMENT_CAN_IDS[] = { 1, 2, 3, 4 };
    private static final int MODULE_ROTATION_CAN_IDS[] = { 5, 6, 7, 8 };
    private static final int MODULE_CANCODER_CAN_IDS[] = { 9, 10, 11, 12 };
    
    private static final double MODULE_TURN_OFFSETS[] = { 45, -45, 45, -45 };        // Degrees to turn for spinning.
    private static final double MODULE_CANCODER_OFFSETS[] = { -251.6307 + 90.0, -44.1210 + 90.0, -192.0409 + 90.0, -175.1659 + 90.0 };  // Degrees offset between encoder's 0 and module's 0.
    //private static final double MODULE_CANCODER_OFFSETS[] = { 0, 0, 0, 0 };
    private static final double MODULE_COEFFIENTS[] = { -1.0, -1.0, -1.0, -1.0 };    // Multiplier to the output of angular PID.

    private static final boolean MODULE_IS_RIGHT[] = { true, false, false, true };

    private static final SwerveDrive instance = new SwerveDrive(MODULE_MOVEMENT_CAN_IDS, MODULE_ROTATION_CAN_IDS, MODULE_CANCODER_CAN_IDS, MODULE_CANCODER_OFFSETS);

    private final SwerveModule modules[] = new SwerveModule[4];
    private final SwerveDriveKinematics kinematics;
    private SwerveMode mode = SwerveMode.Headless;
    private double currentDirection = 0.0;

    private SwerveDrive(int moduleIDs[], int rotationIDs[], int encoderIDs[], double CANCoderOffsets[]) {
        for (int i = 0; i < moduleIDs.length; i++) {
            modules[i] = new SwerveModule(moduleIDs[i], rotationIDs[i], encoderIDs[i], CANCoderOffsets[i], MODULE_COEFFIENTS[i]);
        }

        kinematics = new SwerveDriveKinematics(
            new Translation2d(0.3, 0.3),
            new Translation2d(-0.3, 0.3),
            new Translation2d(-0.3, -0.3),
            new Translation2d(0.3, -0.3)
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
     */
    public static void run(double directionalX, double directionalY, double turn) {
        ChassisSpeeds chassisSpeeds;

        if (directionalX*directionalX + directionalY*directionalY < 0.1) {
            directionalX = 0.0;
            directionalY = 0.0;
        }

        if (Math.abs(turn) < 0.1) {
            turn = 0.0;
        }

        if (instance.mode == SwerveMode.Headless) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(directionalX, -directionalY, turn, new Rotation2d(Pigeon.getRelativeRotationDegrees() * Math.PI / 180.0));
        } else {
            chassisSpeeds = new ChassisSpeeds(directionalX, -directionalY, turn);
        }

        SwerveModuleState[] moduleStates = instance.kinematics.toSwerveModuleStates(chassisSpeeds);

        for (int i = 0; i < instance.modules.length; i++) {
            instance.modules[i].setDesiredState(moduleStates[i]);
        }

        /* double speed = Math.sqrt(directionalX*directionalX + directionalY*directionalY);
        
        if (speed < 0.15)
            speed = 0.0;

        speed = speed * speed * speed;

        run(directionalX, directionalY, turn, Math.sqrt(directionalX*directionalX + directionalY*directionalY)); */
    }

    /**
     * Runs swerve, behavior changes based on the drive's mode.
     * @param directionalX The X axis of the directional control, between 1 and -1
     * @param directionalY The Y axis of the directional control, between 1 and -1.
     * @param turn A value between 1 and -1 that determines the turning angle.
     * @param speed The speed of the drive, between 1 and -1, negative values run backwards.
     */
    public static void run(double directionalX, double directionalY, double turn, double speed) {
        double directionAngle;

        // This makes a distance of less that 0.15 a deadzone on the joystick,
        // used primiraly to mitigate stick drift or minute accidental 
        // movements.
        if (Math.sqrt(directionalX * directionalX + directionalY * directionalY) > 0.15) {
            directionAngle = Math.atan2(directionalY, directionalX) * 180.0 / Math.PI;
            instance.currentDirection = directionAngle;
        } else {
            directionAngle = instance.currentDirection;
        }

        if (Math.abs(speed) < 0.15)
            speed = 0;

        // Turn, since its either turning or not, doesnt change the angle on a 
        // scale, so turning less that 0.5 wont turn at all. When turning 
        // however the turn value is the motor's speed.
        if (Math.abs(turn) > 0.5) {
            for (int i = 0; i < instance.modules.length; i++) {
                double power = (turn * turn * turn);
                power -= Math.signum(power) * 0.125;
                instance.modules[i].setMovementVector(MODULE_TURN_OFFSETS[i], power * (MODULE_IS_RIGHT[i] ? 1 : -1));
            }

            return;
        }

        // Inverts direction angle.
        directionAngle = Math.abs(directionAngle - 360.0);

        // Offsets by 90 since the joystick's zero angle is to the right, and
        // the swerve modules' is up.
        directionAngle = (directionAngle + 90.0) % 360.0;

        // If we're in headless mode we subract the pigeon's position relative
        // to a previously set zero position from the given angle to create an 
        // angle relative to the pigeon's relative zero.
        if (instance.mode == SwerveMode.Headless) {
            for (int i = 0; i < instance.modules.length; i++) {
                instance.modules[i].setMovementVector(directionAngle - Pigeon.getRelativeRotationDegrees(), speed);
            }

            return;
        }

        // Assuming we had no special modes we just set the given angle and 
        // speed.
        for (int i = 0; i < instance.modules.length; i++) {
            instance.modules[i].setMovementVector(directionAngle, speed);
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
