// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.GenericHIDSim;

/**
 * Manages the recording, playback, and wrapping of controller input.
 */
public class Controls {
    private static final double DEFAULT_CURVE_EXPONENT = 3;

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

        return x * distanceRatio;
    };
}
