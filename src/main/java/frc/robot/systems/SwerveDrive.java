// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems;

import java.security.InvalidParameterException;
import java.util.List;
import java.util.function.BiFunction;
import java.util.function.Function;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

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

    private static final double CHASSIS_SIDE_LENGTH = 0.6;
    private static final double RADIAN_DEGREE_RATIO = Math.PI / 180.0;

    private static final SwerveDrive instance = new SwerveDrive();

    private BiFunction<Double, Double, Double> directionCurve = Controls::defaultCurveTwoDimensional;
    private Function<Double, Double> turnCurve = Controls::defaultCurve;

    private final SwerveModule modules[] = new SwerveModule[4];
    private final SwerveModulePosition positions[] = new SwerveModulePosition[4];
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
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

        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }

        kinematics = new SwerveDriveKinematics(
            new Translation2d(   CHASSIS_SIDE_LENGTH / 2,   CHASSIS_SIDE_LENGTH / 2),
            new Translation2d(  -CHASSIS_SIDE_LENGTH / 2,   CHASSIS_SIDE_LENGTH / 2),
            new Translation2d(  -CHASSIS_SIDE_LENGTH / 2,  -CHASSIS_SIDE_LENGTH / 2),
            new Translation2d(   CHASSIS_SIDE_LENGTH / 2,  -CHASSIS_SIDE_LENGTH / 2)
        );

        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(Pigeon.getFeildRelativeRotation() * RADIAN_DEGREE_RATIO), positions);
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
        instance.odometry.update(new Rotation2d(Pigeon.getFeildRelativeRotation() * RADIAN_DEGREE_RATIO), instance.positions);
        
        if (lowSense != -1) {
            double angle = (2 * Math.PI) - ((double)lowSense * RADIAN_DEGREE_RATIO);

            // inverted since the drive is rotated to compensate for joystick stuff
            directionalX = -(Math.sin(angle) * LOW_SENSITIVITY_RATIO);
            directionalY = -(Math.cos(angle) * LOW_SENSITIVITY_RATIO);
            
            runUncurved(directionalX, directionalY, instance.turnCurve.apply(turn));
            return;
        }

        directionalX = instance.directionCurve.apply(directionalX, directionalY);
        directionalY = instance.directionCurve.apply(directionalY, directionalX);
        turn = instance.turnCurve.apply(turn);

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

    public static SequentialCommandGroup getAutoCommand() {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(0.5, 0.5);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
            List.of(
                new Translation2d(0.0, 1.0),
                new Translation2d(1.0, 1.0)
            ),
            new Pose2d(1.0, 1.0, new Rotation2d(0.0)),
            trajectoryConfig
        );

        PIDController xController = new PIDController(0.05, 0.0, 0.0);
        PIDController yController = new PIDController(0.05, 0.0, 0.0);
        ProfiledPIDController profiledController = new ProfiledPIDController(0.1, 0.0, 0.0, new Constraints(1.0, 1.0));

        SwerveControllerCommand command = new SwerveControllerCommand(
            trajectory, 
            instance.odometry::getPoseMeters, 
            instance.kinematics,
            xController,
            yController,
            profiledController,
            SwerveDrive::setModuleStates
        );

        return new SequentialCommandGroup(
            new InstantCommand(() -> SwerveDrive.resetOdometry(trajectory.getInitialPose())),
            command,
            new InstantCommand(() -> SwerveDrive.runUncurved(0.0, 0.0, 0.0))
        );
    }

    /**
     * Sets all module states.
     */
    private static void setModuleStates(SwerveModuleState[] states) {
        for (int i = 0; i < states.length; i++) {
            instance.modules[i].setDesiredState(states[i]);
        }
    }

    /**
     * resets the odometry position.
     * @param position
     */
    private static void resetOdometry(Pose2d position) {
        instance.odometry.resetPosition(new Rotation2d(Pigeon.getFeildRelativeRotation() * RADIAN_DEGREE_RATIO), instance.positions, position);
    }

    public static void print() {
        for (SwerveModule module : instance.modules) {
            module.print();
        }
    }
}
