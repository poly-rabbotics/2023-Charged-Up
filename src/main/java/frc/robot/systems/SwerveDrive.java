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
    private static final int MODULE_MOVEMENT_CAN_IDS[] = { 1,   2,   3,   4  };
    private static final int MODULE_ROTATION_CAN_IDS[] = { 5,   6,   7,   8  };
    private static final int MODULE_CANCODER_CAN_IDS[] = { 9,   10,  11,  12 };
    
    private static final double CHASSIS_SIDE_LENGTH = 0.6;

    private static final Angle MODULE_CANCODER_OFFSETS[] = {
        new Angle().setDegrees(-252.24607237 + 90.0), 
        new Angle().setDegrees(-224.033203125 + 270.0), 
        new Angle().setDegrees(-11.425719246268272 + 270.0), 
        new Angle().setDegrees(-179.56050113588573 + 90.0) 
    };

    private static final Angle MODULE_ROCK_MODE_POSITIONS[] = { 
        new Angle().setRadians( -Angle.TAU / 8  ), 
        new Angle().setRadians(  Angle.TAU / 8  ), 
        new Angle().setRadians( -Angle.TAU / 8  ), 
        new Angle().setRadians(  Angle.TAU / 8  ) 
    };

    private static final Translation2d MODULE_PHYSICAL_POSITIONS[] = {
        new Translation2d(   CHASSIS_SIDE_LENGTH / 2,   CHASSIS_SIDE_LENGTH / 2  ),
        new Translation2d(  -CHASSIS_SIDE_LENGTH / 2,   CHASSIS_SIDE_LENGTH / 2  ),
        new Translation2d(  -CHASSIS_SIDE_LENGTH / 2,  -CHASSIS_SIDE_LENGTH / 2  ),
        new Translation2d(   CHASSIS_SIDE_LENGTH / 2,  -CHASSIS_SIDE_LENGTH / 2  )
    };

    // Singleton instance.
    private static final SwerveDrive instance = new SwerveDrive();

    // Internally mutable state objects
    private final SwerveModule modules[] = new SwerveModule[MODULE_MOVEMENT_CAN_IDS.length];
    private final SwerveModulePosition positions[] = new SwerveModulePosition[MODULE_MOVEMENT_CAN_IDS.length];
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    // Fully mutable state objects    
    private BiFunction<Double, Double, Double> directionCurve = Controls::defaultCurveTwoDimensional;
    private BiFunction<Double, Double, Double> inactiveDirectionCurve = null;
    private Function<Double, Double> turnCurve = Controls::defaultCurve;
    private Function<Double, Double> inactiveTurnCurve = null;
    private SwerveMode mode = SwerveMode.HEADLESS;
    private SwerveMode inactiveMode = null;
    private SwerveMode displayMode = SwerveMode.HEADLESS;

    private SwerveDrive() {
        super();
        
        for (int i = 0; i < MODULE_MOVEMENT_CAN_IDS.length; i++) {
            modules[i] = new SwerveModule(
                MODULE_MOVEMENT_CAN_IDS[i], 
                MODULE_ROTATION_CAN_IDS[i], 
                MODULE_CANCODER_CAN_IDS[i], 
                MODULE_CANCODER_OFFSETS[i], 
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

    public static SwerveMode getDisplayMode() {
        return instance.displayMode;
    }

    /**
     * May be called before running the swerve drive to temporarily set a mode
     * for that call of a run method.
     * 
     * Each time a run method completes it will change the mode back to what it
     * was when this method was called.
     * 
     * If this method is called more than once before running then the latest
     * given temporary mode will be used, the inactive mode to be reset to after
     * running will not be effected
     */
    public static void tempMode(SwerveMode mode) {
        if (instance.inactiveMode != null) {
            instance.mode = mode;
            return;
        }

        instance.inactiveMode = instance.mode;
        instance.mode = mode;
    }

    /**
     * Exactly like `tempMode` but predicates the temporary mode on the given 
     * boolean condition.
     */
    public static void conditionalTempMode(SwerveMode mode, boolean condition) {
        if (!condition) {
            return;
        }

        tempMode(mode);
    }

    /**
     * Sets the curve function for directional inputs (translations).
     * @param curve The BiFunction to use for proccessing the curve, the first 
     * argument is what should be curved, the second is used for context. Return
     * the curved direction.
     */
    public static void setDirectionalCurve(BiFunction<Double, Double, Double> curve) {
        instance.directionCurve = curve;
    }

    /**
     * Gets the current curve used for directional inputs.
     */
    public static BiFunction<Double, Double, Double> getDirectionalCurve() {
        return instance.directionCurve;
    }

    /**
     * Temporarily sets the curve function for directional inputs (translations).
     * This action will atomatically be undone after calling a run method.
     * @param curve The BiFunction to use for proccessing the curve, the first 
     * argument is what should be curved, the second is used for context. Return
     * the curved direction.
     */
    public static void tempDirectionalCurve(BiFunction<Double, Double, Double> curve) {
        if (instance.inactiveDirectionCurve != null) {
            instance.directionCurve = curve;
            return;
        }

        instance.inactiveDirectionCurve = instance.directionCurve;
        instance.directionCurve = curve;
    }

    /**
     * Exactly like `tempDirectionalCurve` but predicated on a boolean condition.
     */
    public static void conditionalTempDirectionalCurve(
        BiFunction<Double, Double, Double> curve, 
        boolean condition
    ) {
        if (!condition) {
            return;
        }

        tempDirectionalCurve(curve);
    }

    /**
     * Sets the curve function for turn inputs.
     * @param curve The Function to use for proccessing the curve.
     */
    public static void setTurnCurve(Function<Double, Double> curve) {
        instance.turnCurve = curve;
    }

    /**
     * Gets the Function currently used for turning.
     */
    public static Function<Double, Double> getTurnCurve() {
        return instance.turnCurve;
    }

    /**
     * Temporarily sets the curve function for turn inputs. Undone after running.
     * @param curve The Function to use for proccessing the curve.
     */
    public static void tempTurnCurve(Function<Double, Double> curve) {
        if (instance.inactiveTurnCurve != null) {
            instance.turnCurve = curve;
            return;
        }

        instance.inactiveTurnCurve = instance.turnCurve;
        instance.turnCurve = curve;
    }

    /**
     * Exactly like `tempTurnCurve` but predicated on a boolean condition.
     */
    public static void conditionalTempTurnCurve(
        Function<Double, Double> curve, 
        boolean condition
    ) {
        if (!condition) {
            return;
        }

        tempTurnCurve(curve);
    }

    /**
     * Sets whether or not to limit the acceleration of the module.
     */
    public static void setAccelerationRate(double accelerationRate) {
        for (SwerveModule module : instance.modules) {
            module.setAccelerationRate(accelerationRate);
        }
    }

    public static double getAccelerationRate() {
        double rate = 0.0;

        for (SwerveModule module : instance.modules) {
            rate += module.getAccelerationRate();
        }

        return rate / (double)instance.modules.length;
    }

    /**
     * Runs swerve, behavior changes based on the drive's mode. This will reset
     * temporary modes on completion.
     * @param directionalX The X axis of the directional control, between 1 and -1
     * @param directionalY The Y axis of the directional control, between 1 and -1.
     * @param speed The speed scalar for the drive.
     * @param turn A value between 1 and -1 that determines the turning angle.
     * @param lowSense The angle to move in low sensitivity in degrees, -1 for no movement.
     */
    public static void run(
        double directionalX,
        double directionalY,
        double speed,
        double turn
    ) {
        speed = Math.abs(directionalX) <= 0.05 && Math.abs(directionalY) <= 0.05 ? 0.0 : speed;
        
        SmartDashboard.putNumber("direction x - run 0", directionalX);
        SmartDashboard.putNumber("direction y - run 0", directionalY);

        // angle is in radians as per Java's trig methods.
        var angle = Math.atan2(directionalY, directionalX);
        directionalX = Math.cos(angle) * speed;
        directionalY = Math.sin(angle) * speed;

        SmartDashboard.putNumber("direction x - run 1", directionalX);
        SmartDashboard.putNumber("direction y - run 1", directionalY);

        run(directionalX, directionalY, turn);
    }

    /**
     * Runs swerve, behavior changes based on the drive's mode. Derives speed
     * from directional inputs. This will reset temporary modes on completion.
     * @param directionalX The X axis of the directional control, between 1 and -1
     * @param directionalY The Y axis of the directional control, between 1 and -1.
     * @param turn A value between 1 and -1 that determines the turning angle.
     * @param lowSense The angle to move in low sensitivity in degrees, -1 for no movement.
     */
    public static void run(double directionalX, double directionalY, double turn) {
        var x = instance.directionCurve.apply(directionalX, directionalY);
        var y = instance.directionCurve.apply(directionalY, directionalX);
        SmartDashboard.putNumber("direction x - run 2", x);
        SmartDashboard.putNumber("direction y - run 2", y);
        turn = instance.turnCurve.apply(turn);
        runUncurved(x, y, turn);
    }
    
    /**
     * Run the swerve drive exactly by the arguments passed, no curving will 
     * occure on the given inputs, nor will they be deadbanded. This will reset
     * temporary modes on completion.
     * @param directionalX Speed along the x axis. -1.0 - 1.0
     * @param directionalY Speed along the y axis. -1.0 - 1.0
     * @param turn Rate of turn. -1.0 - 1.0
     */
    public static void runUncurved(double directionalX, double directionalY, double turn) {
        SwerveModuleState[] moduleStates = new SwerveModuleState[instance.modules.length];
        boolean holdPos = false;

        switch (instance.mode) {
            case HEADLESS:
                moduleStates = instance.kinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        directionalX, -directionalY, turn, 
                        new Rotation2d(Pigeon.getYaw().radians())
                    )
                );
                break;

            case RELATIVE: 
                moduleStates = instance.kinematics.toSwerveModuleStates(
                    new ChassisSpeeds(directionalX, -directionalY, turn)
                ); 
                break;

            case ROCK: {
                assert moduleStates.length == instance.modules.length;
                holdPos = true;

                for (int i = 0; i < instance.modules.length; i++) {
                    moduleStates[i] = new SwerveModuleState(0.0, new Rotation2d(MODULE_ROCK_MODE_POSITIONS[i].radians()));
                }

                break;
            } 

            // This branch should never be reached as the enum used should never
            // have more than the above possible values.
            default: assert false;
        } 

        for (int i = 0; i < instance.modules.length; i++) {
            instance.modules[i].setDesiredState(moduleStates[i]);
            instance.modules[i].setRockMode(holdPos);
        }

        instance.odometry.update(new Rotation2d(Pigeon.getYaw().radians()), instance.positions);

        // Reset temp state

        instance.displayMode = instance.mode;
        
        if (instance.inactiveMode != null) {
            instance.mode = instance.inactiveMode;
            instance.inactiveMode = null;
        }
        
        if (instance.inactiveDirectionCurve != null) {
            instance.directionCurve = instance.inactiveDirectionCurve;
            instance.inactiveDirectionCurve = null;
        }

        if (instance.inactiveTurnCurve != null) {
            instance.turnCurve = instance.inactiveTurnCurve;
            instance.inactiveTurnCurve = null;
        }
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
     * Gets the average tempurature of all motors on the drive in celsius.
     */
    public static double getAverageMotorTemp() {
        double tempSum = 0.0;

        for (SwerveModule module : instance.modules) {
        	tempSum += module.getRotationMotorTemp();
            tempSum += module.getMovementMotorTemp();
        }

        return tempSum / (instance.modules.length * 2.0);
    }

    /**
     * Gets the sum of all applied currents in amps of all motors on the drive.
     */
    public static double getAppliedCurrent() {
        double current = 0.0;

        for (SwerveModule module : instance.modules) {
        	current += module.getAppliedCurrent();
        }

        return current;
    }

    /**
     * Gets the average percent usage of each module's motor controller 
     * current pull.
     */
    public static double getAveragePercentRatedCurrent() {
        double percentSum = 0.0;

        for (SwerveModule module : instance.modules) {
        	percentSum += module.getPercentRatedCurrent();
        }

        return percentSum / (double)instance.modules.length;
    }

    /**
     * Print data to smart dashboard.
     */
    @Override
    public void print() {
        SmartDashboard.putString("Swerve Drive Mode", getMode().toString());
        SmartDashboard.putString("Swerve Drive Odometry", 
            "(" 
            + ((double)(long)(odometry.getPoseMeters().getX() * 100)) / 100
            + ", "
            + ((double)(long)(odometry.getPoseMeters().getY() * 100)) / 100
            + ") "
            + ((double)(long)(odometry.getPoseMeters().getRotation().getDegrees() * 100)) / 100
            + " degrees"
        );
        SmartDashboard.putNumber("Swerve Drive Average Motor Tempurature (Celsius)", getAverageMotorTemp());
        SmartDashboard.putNumber("Swerve Drive Total Current Pull (Amps)", getAppliedCurrent());
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
