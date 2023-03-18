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
    // Should be the first USB device plugged into the RoboRio. This device 
    // should be formatted as a FAT filesystem with a single partition.
    private static final String RECORDING_PATH = "/media/sda1";
    
    // 50 hz times 60 seconds times 15 seconds in auto
    private static final int RECORDING_FRAMES = 50 * 60 * 15;

    public static final int CONTROLLER_PORT_DRIVE = 0;
    public static final int CONTROLLER_PORT_MECHS = 1;
    public static final int CONTROLLER_PORT_PANEL = 2;

    private static final Controls instance = new Controls();

    /**
     * Represents a single frame of controller input and is capable of setting
     * said state to a controller simulation.
     */
    private class ControllerState {
        private int pov;
        private double[] rawAxes;
        private boolean[] rawButtons;
        private boolean[] rawButtonsReleased;
        private boolean[] rawButtonsPressed;

        /**
         * Updates the controller state with the given controller.
         */
        private void update(GenericHID controller) {
            pov                 = controller.getPOV();
            rawAxes             = new double[controller.getAxisCount()];
            rawButtons          = new boolean[controller.getButtonCount()];
            rawButtonsReleased  = new boolean[controller.getButtonCount()];
            rawButtonsPressed   = new boolean[controller.getButtonCount()];

            for (int i = 0; i < rawAxes.length; i++) {
                rawAxes[i] = controller.getRawAxis(i);
            }

            for (int i = 0; i < rawButtons.length; i++) {
                rawButtons[i]           = controller.getRawButton(i);
                rawButtonsReleased[i]   = controller.getRawButtonReleased(i);
                rawButtonsPressed[i]    = controller.getRawButtonPressed(i);
            }
        }

        /**
         * Set this state to the given controller simulation.
         */
        private void set(GenericHIDSim controller) {
            controller.setAxisCount(rawAxes.length);
            controller.setButtonCount(rawButtons.length);
            controller.setPOV(pov);

            for (int i = 0; i < rawAxes.length; i++) {
                controller.setRawAxis(i, rawAxes[i]);
            }

            for (int i = 0; i < rawButtons.length; i++) {
                controller.setRawButton(i, rawButtons[i]);
            }

            controller.notifyNewData();
        }

        /**
         * Get a string that represents this frame of controller input can be
         * used to restore this state.
         */
        private String getString() {
            String str = Integer.toString(pov) + " ";

            for (double axis : rawAxes) {
                str += Double.toString(axis) + " ";
            }

            for (boolean button : rawButtons) {
                str += Boolean.toString(button) + " ";
            }

            str += "\n";
            return str;
        }

        /**
         * Sets the current state to one frame of the given string, seperated by
         * newlines.
         * @param str The string containing newline seperated input frames.
         * @param frame The frame to set this state to.
         */
        private void setFromString(String str, int frame) {
            String[] frames = str.split("\n");

            // May have to get a regex for this.
            String[] values = frames[frame].split(" ");
            pov = Integer.parseInt(values[0]);

            for (int i = 1; i < 1 + rawAxes.length; i++) {
                rawAxes[i] = Double.parseDouble(values[i]);
            }

            for (int i = 1 + rawAxes.length; i < values.length; i++) {
                rawButtons[i] = Boolean.parseBoolean(values[i]);
            }
        }
    }

    private final GenericHID[] controllers;
    private final GenericHIDSim[] controllerSims;
    private final ControllerState[] controllerStates;
    private String[] controllerRecordings;

    private int mode = 0;
    private int frame = -1;

    private Controls() {
        controllers = new GenericHID[] {
            new XboxController(CONTROLLER_PORT_DRIVE),
            new XboxController(CONTROLLER_PORT_MECHS),
            new Joystick(CONTROLLER_PORT_PANEL)
        };

        controllerSims = new GenericHIDSim[controllers.length];
        controllerStates = new ControllerState[controllers.length];

        for (int i = 0; i < controllers.length; i++) {
            controllerSims[i] = new GenericHIDSim(controllers[i]);
            controllerStates[i] = new ControllerState();
        }
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
     * Writes the current state recordings to files.
     */
    private void writeStates() {
        for (int i = 0; i < controllerRecordings.length; i++) {
            try {
                BufferedWriter writer = new BufferedWriter(
                    new FileWriter(RECORDING_PATH + "/mode" + Integer.toString(mode) + "controller" + Integer.toString(i)));

                writer.write(instance.controllerRecordings[i]);
                writer.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    /**
     * Updates the controller state, this gets new controller data and if 
     * recording records that frame.
     */
    public static void update() {
        if (instance.frame >= RECORDING_FRAMES) {
            instance.frame = -1;
            instance.writeStates();
        }

        if (instance.frame != -1) {
            for (int i = 0; i < instance.controllers.length; i++) {
                instance.controllerStates[i].update(instance.controllers[i]);
                instance.controllerRecordings[i] += instance.controllerStates[i].getString();
            }

            instance.frame++;
        }

        for (int i = 0; i < instance.controllers.length; i++) {
            instance.controllerStates[i].update(instance.controllers[i]);
        }
    }

    /**
     * Starts a fifteen second recording. At the end of those fifteen seconds
     * the recording is written to files, one for each controller.
     * 
     * NOTE: This methods assumes assumes its being called at 50hz for those
     * fifteen seconds.
     * 
     * @param mode The auto mode to record to. 
     */
    public static void startRecording(int mode) {
        instance.frame = 0;
        instance.controllerRecordings = new String[instance.controllerStates.length];

        for (int i = 0; i < instance.controllerRecordings.length; i++) {
            instance.controllerRecordings[i] = "";
        }
    }

    /**
     * plays back a recording given the auto mode it was recorded to.
     * @param mode the mode to play back
     */
    public static void playback(int mode) {
        if (instance.frame == -1) {
            instance.frame = 0;
        }

        if (instance.frame >= RECORDING_FRAMES) {
            return;
        }

        if (instance.controllerRecordings == null) {
            instance.controllerRecordings = new String[instance.controllers.length];

            for (int i = 0; i < instance.controllers.length; i++) {
                try {
                    instance.controllerRecordings[i] = new String(Files.readAllBytes(Paths.get(RECORDING_PATH + "/mode" + Integer.toString(mode) + "controller" + Integer.toString(i))));
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }

        for (int i = 0; i < instance.controllerRecordings.length; i++) {
            instance.controllerStates[i].setFromString(instance.controllerRecordings[i], instance.frame);
            instance.controllerStates[i].set(instance.controllerSims[i]);
        }

        instance.frame++;
    }
}