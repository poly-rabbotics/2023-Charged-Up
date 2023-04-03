package frc.robot.systems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.patterns.Breathe;

/**
 * Manages the SwerDrive class for auto balancing, this is only seperate since
 * SwerveDrive was becoming a bit large to handle.
 */
public class AutoBalance {
    private static final double RAMMING_SPEED = -0.5;
    private static final double CLIMBING_SPEED = -0.3;
    private static final double ENCROACHING_SPEED = -0.1;
    private static final double HALTING_SPEED = 0.1;

    private static final double ANGLE_SPIKE_QUALIFIER = 20.0;
    private static final double ANGLE_STABLILITY_QUALIFIER = 5.0;
    
    private static final AutoBalance instance = new AutoBalance();

    private Stage stage;
    private boolean halting = false;

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

        if (Math.abs(SwerveDrive.getModulePos(0)) > 330) {
            instance.halting = true;
        }

        /* if (Math.abs(SwerveDrive.getModulePos(0)) < TICKS_OVER && !instance.over) {
            instance.halting = false;
        } else if (!instance.over) {
            instance.over = true;
            SwerveDrive.zeroPositions();
        } else if (Math.abs(SwerveDrive.getModulePos(0)) < TICKS_OVER_BALANCE) {
            instance.halting = false;
            SwerveDrive.runUncurved(0.0, -RAMMING_SPEED, 0.0);
            return;
        } else {
            instance.halting = true;
        } */

        if (instance.halting) {
            instance.stage = Stage.HALTING;
        } else {
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
        instance.halting = stage == Stage.HALTING;
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
/*         // Negate for downward spike.
        if (Math.abs(Pigeon.getChangePerSecond().pitchPerSecond) > ANGLE_SPIKE_QUALIFIER) {
            stage = Stage.HALTING;
            return;
        } */

        LEDLights.setPatternIfNotEqual(new Breathe(new Color(0.0, 0.0, 1.0), 5.0));
        SwerveDrive.runUncurved(0.0, ENCROACHING_SPEED, 0.0);
    }

    private void halt() {
        if (Math.abs(Pigeon.getPitch()) < 10.0 && Math.abs(Pigeon.getChangePerSecond().pitchPerSecond) < 10.0) {
            SwerveDrive.runUncurved(0.0, 0.0, 0.0);
            return;
        }

        if (Math.abs(Pigeon.getChangePerSecond().pitchPerSecond) > 10) {
            SwerveDrive.runUncurved(0.0, 0.0, 0.0);
            return;
        }

        LEDLights.setPatternIfNotEqual(new Breathe(new Color(0.0, 1.0, 0.0), 5.0));
        SwerveDrive.runUncurved(0.0, Math.signum(Pigeon.getPitch()) * HALTING_SPEED, 0.0);
    }

    public static void print() {
        SmartDashboard.putString("Auto Balance Stage: ", instance.stage.toString());
    }
}
