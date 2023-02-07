// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems;

import frc.robot.subsystems.SwerveMode;
import frc.robot.subsystems.SwerveModule;

public class SwerveDrive {
    private static final int MODULE_MOVEMENT_CAN_IDS[] = { 1, 2, 3, 4 };
    private static final int MODULE_ROTATION_CAN_IDS[] = { 5, 6, 7, 8 };
    private static final int MODULE_ENCODER_CAN_IDS[] = { 5, 6, 7, 8 };

    private static SwerveDrive instance = new SwerveDrive(MODULE_MOVEMENT_CAN_IDS, MODULE_ROTATION_CAN_IDS, MODULE_ENCODER_CAN_IDS);

    private SwerveMode mode = SwerveMode.Relative;
    private SwerveModule modules[];

    private SwerveDrive(int moduleIDs[], int rotationIDs[], int encoderIDs[]) {
        for (int i = 0; i < moduleIDs.length; i++) {
            modules[i] = new SwerveModule(moduleIDs[i], rotationIDs[i], encoderIDs[i]);
        }
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
     * If mode is Headless then the given rotation will be assumed to be
     * relative to the driver.
     * 
     * @param rotation Rotation in degrees.
     * @param speed Native motor speed.
     */
    public static void setMovementVector(double rotation, double speed) {
        if (instance.mode == SwerveMode.Relative) {
            for (SwerveModule module : instance.modules) {
                module.setMovementVector(rotation, speed);
            }

            return;
        }
    
        double orientation = Pigeon.getAbsoluteRotationDegrees();

        for (SwerveModule module : instance.modules) {
            module.setMovementVector(rotation - orientation, speed);
        }        
    }
}
