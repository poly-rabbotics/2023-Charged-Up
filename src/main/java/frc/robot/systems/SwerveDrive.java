// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems;

import frc.robot.subsystems.SwerveMode;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.wpilibj.XboxController;

public class SwerveDrive {
    private static final int MODULE_MOVEMENT_CAN_IDS[] = { 1, 6, 3, 8 };
    private static final int MODULE_ROTATION_CAN_IDS[] = { 10, 12, 14, 13 };
    private static final int MODULE_ENCODER_CAN_IDS[] = { 5, 9, 7, 8 };

    private static final double MODULE_CANCODER_OFFSETS[] = {-12, 25, -113, 40}; //degrees offset between encoder's 0 and module's 0

    private static double testRotation = 0;
    private static double testSpeed = 0;

    private static SwerveDrive instance = new SwerveDrive(MODULE_MOVEMENT_CAN_IDS, MODULE_ROTATION_CAN_IDS, MODULE_ENCODER_CAN_IDS, MODULE_CANCODER_OFFSETS);
    public static XboxController testController = new XboxController(0);

    private SwerveMode mode = SwerveMode.Relative;
    private SwerveModule modules[] = new SwerveModule[4];

    private SwerveDrive(int moduleIDs[], int rotationIDs[], int encoderIDs[], double CANCoderOffsets[]) {
        for (int i = 0; i < moduleIDs.length; i++) {
            modules[i] = new SwerveModule(moduleIDs[i], rotationIDs[i], encoderIDs[i], CANCoderOffsets[i]);
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
    
        double orientation = Pigeon.getRelativeRotationDegrees();

        for (SwerveModule module : instance.modules) {
            module.setMovementVector(rotation - orientation, speed);
        }        
    }

    /**
     * Temporary method to test functionality of swerve modules
     */
    public static void test() {
        resetOrientation();
        for(SwerveModule module : instance.modules) {
            module.adjustPIDs();
        }
        /* 
        testRotation = testController.getRawAxis(4);
        testSpeed = testController.getRawAxis(1);
            if (Math.abs(testRotation) < 0.1) testRotation = 0;
            if (Math.abs(testSpeed) < 0.1) testSpeed = 0;
            

        for(SwerveModule module : instance.modules) {
            module.setMovementVector(testRotation, testSpeed);
        } */
    }
}
