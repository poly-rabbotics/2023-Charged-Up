// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems;

import java.util.function.BiFunction;
import java.util.function.Function;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Manages the recording, playback, and wrapping of controller input.
 */
public class Controls {
    private static final double DEFAULT_CURVE_EXPONENT = 3;
    private static final double DEFAULT_PLATEAU = 0.15;

    public static final int CONTROLLER_PORT_DRIVE = 0;
    public static final int CONTROLLER_PORT_MECHS = 1;
    public static final int CONTROLLER_PORT_PANEL = 2;

    private static final Controls instance = new Controls();

    private final GenericHID[] controllers;

    private Controls() {
        controllers = new GenericHID[] {
            new XboxController(CONTROLLER_PORT_DRIVE),
            new XboxController(CONTROLLER_PORT_MECHS),
            new Joystick(CONTROLLER_PORT_PANEL)
        };
    }

    /**
     * gets a controller by port. returns null if no controller is found.
     */
    public static GenericHID getControllerByPort(int port) {
        for (GenericHID controller : instance.controllers) {
            if (controller.getPort() == port) {
                return controller;
            }
        }

        return null;
    }

    /**
     * Default one dimensional curve.
     */
    public static double defaultCurve(double x) {
        if (Math.abs(x) < 0.1) {
            return 0.0;
        }

        return Math.pow(x, DEFAULT_CURVE_EXPONENT);
    }

    public static double invertedCurve(double x) {
        return -defaultCurve(x);
    }

    /**
     * Custom turning curve for Rohan.
     */
    public static double turnCurveRohan(double x) {
        if (Math.abs(x) < 0.1) {
            return 0.0;
        }

        return Math.pow(x, 5);
    }

    /**
     * Default two dimensional curve, returns the curved x value, y is only
     * used for context. Swap parameters to get the other value curved.
     * @param x The value to be curved.
     * @param y The value used for two dimensional context.
     * @return The curved x value.
     */
    public static double defaultCurveTwoDimensional(double x, double y) {
        double distance = Math.sqrt(x*x + y*y);

        if (distance < 0.1) {
            return 0.0;
        }

        double curvedDistance = Math.pow(distance, DEFAULT_CURVE_EXPONENT);
        double distanceRatio = curvedDistance * distance;

        return Math.sqrt(x * x * distanceRatio) * Math.signum(x);
    }

    /**
     * Returns `x` if `x` is greater than `y`, otherwise returning `0.0`.
     */
    public static double cardinalLock(double x, double y) {
        return Math.abs(x) > Math.abs(y) 
            ? x
            : 0.0;
    }

    /**
     * Applies a cardinal lock to inputs of the given curve.
     */
    public static BiFunction<Double, Double, Double> cardinalLock(
        BiFunction<Double, Double, Double> curve
    ) {
        return (x, y) -> {
            x = cardinalLock(x, y);
            y = cardinalLock(y, x);
            return curve.apply(x, y);
        };
    }

    /**
     * Runs th default curve with a plateau.
     */
    public static double plateauingCurve(double x) {
        return plateau(x, Controls::defaultCurve);
    }

    /**
     * Runs the default 2D curve with a plateau.
     */
    public static double plateauingCurveTwoDimensional(double x, double y) {
        return plateauTwoDimensional(x, y, Controls::defaultCurveTwoDimensional);
    }

    /**
     * Plateaus the given value and runs it through the given curve.
     */
    public static double plateau(double x, Function<Double, Double> curve) {
        x = x + (x * DEFAULT_PLATEAU);
        x = x > 1.0 ? 1.0 : x;
        return curve.apply(x);
    }

    /**
     * Plateaus the given value and runs it through the given curve.
     */
    public static double plateauTwoDimensional(double x, double y, BiFunction<Double, Double, Double> curve) {
        double magnitude = Math.sqrt(x*x + y*y);
        magnitude = magnitude + (magnitude * DEFAULT_PLATEAU);
        magnitude = magnitude > 1.0 ? 1.0 : magnitude;
        return curve.apply(x * magnitude, y * magnitude);
    }
}
