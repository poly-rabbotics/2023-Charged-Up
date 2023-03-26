package frc.robot.systems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.patterns.Breathe;

/**
 * Manages the SwerDrive class for auto balancing, this is only seperate since
 * SwerveDrive was becoming a bit large to handle.
 */
public class AutoBalance {
    private static final double RAMMING_SPEED = -0.75;
    private static final double CLIMBING_SPEED = -0.5;
    private static final double ENCROACHING_SPEED = -0.1;
    private static final double HALTING_SPEED = 0.1;

    private static final double ANGLE_SPIKE_QUALIFIER = 10.0;
    private static final double ANGLE_STABLILITY_QUALIFIER = 5.0;
    private static final int HALTING_FRAMES = 5;
    
    private static final AutoBalance instance = new AutoBalance();

    private Stage stage;
    private int haltedFrames;

    public enum Stage {
        IDLING, 
        RAMMING,
        CLIMBING,
        ENCROACHING, 
        HALTING, 
        DONE
    }

    private AutoBalance() {
        stage = Stage.IDLING;
        haltedFrames = 0;
    }

    /**
     * Call in a loop once aimed at the charge station to run auto balance. Get
     * the current stage and check for "DONE" to see if it has completed.
     */
    public static void run() {
        if (instance.stage == Stage.DONE) {
            SwerveDrive.runUncurved(0.0, 0.0, 0.0);
            return;
        }

        if (instance.stage == Stage.IDLING) {
            instance.stage = Stage.RAMMING;
        }

        switch (instance.stage) {
            case RAMMING:
                instance.ram();
                break;
            
            case CLIMBING:
                instance.climb();
                break;

            case ENCROACHING:
                instance.encroach();
                break;
            
            case HALTING:
                instance.halt();
                break;
            
            default:
                throw new RuntimeException("AutoBalance stage was misshandled. Value: " + instance.stage.toString());
        }
    }

    /**
     * Gets the auto balance stage.
     */
    public static Stage getStage() {
        return instance.stage;
    }

    public static void setStage(Stage stage) {
        instance.stage = stage;
    }

    private void ram() {
        // Detects upward angle spike and advances the stage.
        if (Pigeon.getChangePerSecond().pitchPerSecond > ANGLE_SPIKE_QUALIFIER) {
            stage = Stage.CLIMBING;
            return;
        }

        SwerveDrive.runUncurved(0.0, RAMMING_SPEED, 0.0);
        LEDLights.setPatternIfNotEqual(new Breathe(new Color(1.0, 0.0, 0.0), 5.0));
    }

    private void climb() {
        // Waits for the rate of change to stabilize.
        if (Math.abs(Pigeon.getChangePerSecond().pitchPerSecond) < ANGLE_STABLILITY_QUALIFIER) {
            stage = Stage.ENCROACHING;
            return;
        }

        LEDLights.setPatternIfNotEqual(new Breathe(new Color(1.0, 0.0, 1.0), 5.0));
        SwerveDrive.runUncurved(0.0, CLIMBING_SPEED, 0.0);
    }

    private void encroach() {
        // Negate for downward spike.
        if (Pigeon.getChangePerSecond().pitchPerSecond < -ANGLE_SPIKE_QUALIFIER) {
            stage = Stage.HALTING;
            return;
        }

        LEDLights.setPatternIfNotEqual(new Breathe(new Color(0.0, 0.0, 1.0), 5.0));
        SwerveDrive.runUncurved(0.0, ENCROACHING_SPEED, 0.0);
    }

    private void halt() {
        if (haltedFrames > HALTING_FRAMES) {
            stage = Stage.DONE;
            return;
        }

        LEDLights.setPatternIfNotEqual(new Breathe(new Color(0.0, 1.0, 0.0), 5.0));
        SwerveDrive.runUncurved(0.0, HALTING_SPEED, 0.0);
        haltedFrames++;
    }

    public static void print() {
        SmartDashboard.putString("Auto Balance Stage: ", instance.stage.toString());
    }
}
