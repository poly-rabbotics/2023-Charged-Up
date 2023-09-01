// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems;

import java.security.InvalidParameterException;

import java.util.function.BiFunction;
import java.util.function.Function;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SmartPrintable;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.SwerveMode;
import frc.robot.subsystems.SwerveModule;

/*
 * Manages the swerve drive train.
 */
public class SwerveDrive extends SmartPrintable {
    private static final int MODULE_MOVEMENT_CAN_IDS[] = { 1,  2,   3,   4  };
    private static final int MODULE_ROTATION_CAN_IDS[] = { 5,  6,   7,   8  };
    private static final int MODULE_CANCODER_CAN_IDS[] = { 9,  10,  11,  12 };
    
    private static final double MODULE_COEFFIENTS[] = { -1.0, -1.0, -1.0, -1.0 };
    private static final double LOW_SENSITIVITY_RATIO = 0.08;
    private static final double CHASSIS_SIDE_LENGTH = 0.6;

    private static final Angle MODULE_CANCODER_OFFSETS[] = {
        new Angle().setDegrees(-252.24607237 + 90.0), 
        new Angle().setDegrees(-224.033203125 + 270.0), 
        new Angle().setDegrees(-11.425719246268272 + 270.0), 
        new Angle().setDegrees(-179.56050113588573 + 90.0) 
    };

    private static final Angle MODULE_ROCK_MODE_POSITIONS[] = { 
        new Angle().setRadians( -Math.PI / 4), 
        new Angle().setRadians(  Math.PI / 4), 
        new Angle().setRadians( -Math.PI / 4), 
        new Angle().setRadians(  Math.PI / 4) 
    };

    private static final Translation2d MODULE_PHYSICAL_POSITIONS[] = {
        new Translation2d(   CHASSIS_SIDE_LENGTH / 2,   CHASSIS_SIDE_LENGTH / 2),
        new Translation2d(  -CHASSIS_SIDE_LENGTH / 2,   CHASSIS_SIDE_LENGTH / 2),
        new Translation2d(  -CHASSIS_SIDE_LENGTH / 2,  -CHASSIS_SIDE_LENGTH / 2),
        new Translation2d(   CHASSIS_SIDE_LENGTH / 2,  -CHASSIS_SIDE_LENGTH / 2)
    };

    // Singleton instance.
    private static final SwerveDrive instance = new SwerveDrive();

    // These drive state objects modify themselves internally through methods,
    // but require no setting and are therefore final.
    private final SwerveModule modules[] = new SwerveModule[MODULE_MOVEMENT_CAN_IDS.length];
    private final SwerveModulePosition positions[] = new SwerveModulePosition[MODULE_MOVEMENT_CAN_IDS.length];
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    
    // Variable drive states objects.
    private BiFunction<Double, Double, Double> directionCurve = Controls::defaultCurveTwoDimensional;
    private Function<Double, Double> turnCurve = Controls::defaultCurve;
    private SwerveMode mode = SwerveMode.HEADLESS;

    private SwerveDrive() {
        super();
        
        for (int i = 0; i < MODULE_MOVEMENT_CAN_IDS.length; i++) {
            modules[i] = new SwerveModule(
                MODULE_MOVEMENT_CAN_IDS[i], 
                MODULE_ROTATION_CAN_IDS[i], 
                MODULE_CANCODER_CAN_IDS[i], 
                MODULE_CANCODER_OFFSETS[i], 
                MODULE_COEFFIENTS[i],
                MODULE_PHYSICAL_POSITIONS[i]
            );
        }

        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }

        kinematics = new SwerveDriveKinematics(
            MODULE_PHYSICAL_POSITIONS[0],
            MODULE_PHYSICAL_POSITIONS[1],
            MODULE_PHYSICAL_POSITIONS[2],
            MODULE_PHYSICAL_POSITIONS[3]
        );

        odometry = new SwerveDriveOdometry(
            kinematics, 
            new Rotation2d(Pigeon.getYaw().radians()), 
            positions
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
     * Runs swerve, behavior changes based on the drive's mode. Derives speed
     * from directional inputs.
     * @param directionalX The X axis of the directional control, between 1 and -1
     * @param directionalY The Y axis of the directional control, between 1 and -1.
     * @param turn A value between 1 and -1 that determines the turning angle.
     * @param lowSense The angle to move in low sensitivity in degrees, -1 for no movement.
     */
    public static void run(double directionalX, double directionalY, double turn, int lowSense) {
        if (getRockMode()) {
            runRockMode();
            return;
        }
        
        instance.odometry.update(new Rotation2d(Pigeon.getYaw().radians()), instance.positions);
        
        if (lowSense != -1) {
            double angle = Angle.TAU - Math.toRadians((double)lowSense);

            // inverted since the drive is rotated to compensate for joystick stuff
            directionalX = -(Math.sin(angle) * LOW_SENSITIVITY_RATIO);
            directionalY = -(Math.cos(angle) * LOW_SENSITIVITY_RATIO);
            
            runUncurved(directionalX, directionalY, instance.turnCurve.apply(turn));
            return;
        }

        directionalX = instance.directionCurve.apply(directionalX, directionalY);
        directionalY = instance.directionCurve.apply(directionalY, directionalX);
        turn = instance.turnCurve.apply(turn);

        ChassisSpeeds chassisSpeeds = instance.mode == SwerveMode.HEADLESS
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                directionalX, -directionalY, turn, 
                new Rotation2d(Pigeon.getYaw().radians())
            )
            : new ChassisSpeeds(directionalX, -directionalY, turn);

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
        if (getRockMode()) {
            runRockMode();
            return;
        }

        ChassisSpeeds chassisSpeeds = instance.mode == SwerveMode.HEADLESS
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                directionalX, -directionalY, turn, 
                new Rotation2d(Pigeon.getYaw().radians())
            )
            : new ChassisSpeeds(directionalX, -directionalY, turn);

        SwerveModuleState[] moduleStates = instance.kinematics.toSwerveModuleStates(chassisSpeeds);

        for (int i = 0; i < instance.modules.length; i++) {
            instance.modules[i].setDesiredState(moduleStates[i]);
        }
    }

    /**
     * Wyvern becomes rock, rock do not move, Wyvern do not move...
     */
    public static void runRockMode() {
        for (int i = 0; i < instance.modules.length; i++) {
            instance.modules[i].setDesiredState(new SwerveModuleState(0.0, new Rotation2d(MODULE_ROCK_MODE_POSITIONS[i].radians())));
        }
    }

    /**
     * True if rock mode should be active.
     */
    public static void setRockMode(boolean shouldHold) {
        for (SwerveModule module : instance.modules) {
            module.setRockMode(shouldHold);
        }
    }

    /**
     * Returns true if in rock mode.
     */
    public static boolean getRockMode() {
        boolean rockMode = false;

        for (SwerveModule module : instance.modules) {
            rockMode |= module.inRockMode();
        }

        return rockMode;
    }

    /**
     * Sets the max speed of all modules, use NaN for no limit.
     */
    public static void setMaxSpeed(double maxSpeed) {
        for (SwerveModule module : instance.modules) {
            module.setMaxSpeed(maxSpeed);
        }
    }

    /**
     * Sets the curve function for directional inputs (translations).
     * @param curve The BiFunction to use for proccessing the curve, the first 
     * argument is what should be curved, the second is used for context. Return
     * the curved direction.
     */
    public static void setDirectionalCurve(BiFunction<Double, Double, Double> curve) {
        // Run some tests that should be valid of any curve function.
        if (curve.apply(0.0, 0.0) != 0.0) {
            throw new InvalidParameterException("All curves should return 0 on a 0 input.");
        }

        double previousOutput = 0.0;

        // Here we just make sure all outputs either go up or stay the same as
        // the inputs become greator.
        for (double i = 0.0; i < 1.0; i += 0.1) {
            double outI = curve.apply(i, i);

            if (outI < previousOutput) {
                throw new InvalidParameterException("No curve shall return a lesser value given a greator input.");
            }

            for (double j = 0.0; j < 1.0; j += 0.1) {
                double outJ = curve.apply(i, j);

                if (j < i && outJ > outI) {
                    throw new InvalidParameterException("No curve shall return a greator value given a lesser input.");
                } else if (j > i && outJ < outI) {
                    throw new InvalidParameterException("No curve shall return a lesser value given a greator input.");
                } else if (j == i && outJ != outI) {
                    throw new InvalidParameterException("No curve shall return a different result given the same input (i mean, c'mon man, thats the deffinition of a function...).");
                }
            }

            previousOutput = outI;
        }

        instance.directionCurve = curve;
    }

    public static void setDirectionalCurve(BiFunction<Double, Double, Double> curve, boolean overrideTests) {
        if (overrideTests) {
            instance.directionCurve = curve;
            return;
        }

        setDirectionalCurve(curve);
    }

    /**
     * Sets the curve function for turn inputs.
     * @param curve The Function to use for proccessing the curve.
     */
    public static void setTurnCurve(Function<Double, Double> curve) {
        // Run some tests that should be valid of any curve function.
        if (curve.apply(0.0) != 0.0) {
            throw new InvalidParameterException("All curves should return 0 on a 0 input.");
        }

        double previousOutput = 0.0;

        // Here we just make sure all outputs either go up or stay the same as
        // the inputs become greator.
        for (double i = 0.0; i < 1.0; i += 0.1) {
            double outI = curve.apply(i);

            if (outI < previousOutput) {
                throw new InvalidParameterException("No curve shall return a lesser value given a greator input.");
            }

            previousOutput = outI;
        }

        instance.turnCurve = curve;
    }

    /**
     * Print data to smart dashboard.
     */
    @Override
    public void print() {
        SmartDashboard.putString("Orientation", getMode().toString());
        SmartDashboard.putBoolean("In Rock Mode", getRockMode());
    }

    /**
     * Gets the longest distance traveled by any modules.
     */
    public static double getDistance() {
        return Math.max(Math.abs(instance.modules[0].getDistanceTraveled()), Math.abs(instance.modules[1].getDistanceTraveled()));
    }

    /**
     * Zeros all movement encoder positions.
     */
    public static void zeroPositions() {
        for (SwerveModule module : instance.modules) {
            module.zeroPositions();
        }
    }
}
