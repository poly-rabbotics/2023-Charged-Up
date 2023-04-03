package frc.robot.systems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.patterns.Breathe;

/**
 * Manages the SwerDrive class for auto balancing, this is only seperate since
 * SwerveDrive was becoming a bit large to handle.
 */
public class AutoBalance {
    private static final double RAMMING_SPEED = -0.5;
    private static final double HALTING_SPEED = 0.1;
    
    private static final AutoBalance instance = new AutoBalance();

    private Stage stage;

    public enum Stage {
        /** auto balance is doing nothing. */
        IDLING,

        /** auto balance is ramming, meant to dock the charge station. */

        RAMMING,

        /** auto balance is on the charge station and trying to engage the charge station. */
        ADJUSTING,
        
        /** auto balance has paused, either becaue it has finished, or because it has encountered a spike in angular change. */
        PAUSED,
    }

    private AutoBalance() {
        stage = Stage.IDLING;
    }

    /**
     * Call in a loop during autonomous to auto balance, auto balance expects 
     * to be the last action taken during autonomous and other actions affecting
     * the drive train should not be taken.
     */
    public static void run() {
        if (instance.stage == Stage.IDLING) {
            instance.stage = Stage.RAMMING;
        }

        switch (instance.stage) {
            case RAMMING:
                instance.ram();
                break;  
            
            case ADJUSTING:
                instance.adjust();
                break;
            
            case PAUSED:
                instance.pause();
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

    /**
     * Manually sets the stage of auto balance.
     */
    public static void setStage(Stage stage) {
        instance.stage = stage;
    }

    private Stage ram() {
        if (Math.abs(SwerveDrive.getModulePos(0)) > 330) {
            return adjust();
        }

        SwerveDrive.runUncurved(0.0, RAMMING_SPEED, 0.0);
        LEDLights.setPatternIfNotEqual(new Breathe(new Color(1.0, 0.0, 0.0), 5.0));
        return Stage.RAMMING;
    }

    // Hopefully achieves a similar result to the distance based ram but from 
    // any start point given the orientation of the bot.
    private Stage dynamicRam() {
        if (spiking() & !balanced()) {
            return adjust();
        }

        SwerveDrive.runUncurved(0.0, RAMMING_SPEED, 0.0);
        LEDLights.setPatternIfNotEqual(new Breathe(new Color(1.0, 0.0, 0.0), 5.0));
        return Stage.RAMMING;
    }

    private Stage adjust() {
        if (balanced() | spiking()) {
            return pause();
        }

        SwerveDrive.runUncurved(0.0, Math.signum(Pigeon.getPitch()) * HALTING_SPEED, 0.0);
        LEDLights.setPatternIfNotEqual(new Breathe(new Color(1.0, 0.0, 1.0), 5.0));
        return Stage.ADJUSTING;
    }

    private Stage pause() {
        if (!balanced()) {
            return instance.stage = adjust();
        }

        SwerveDrive.runUncurved(0.0, 0.0, 0.0);
        LEDLights.setPatternIfNotEqual(new Breathe(new Color(1.0, 0.0, 1.0), 5.0));
        return Stage.PAUSED;
    }

    /**
     * Returns true if auto balance beleives it is balanced, not 100% accurate.
     */
    private boolean balanced() {
        return Math.abs(Pigeon.getPitch()) < 10.0 && Math.abs(Pigeon.getChangePerSecond().pitchPerSecond) < 10.0;
    }

    /**
     * Returns true if auto balance beleives it is spiking in angle.
     */
    private boolean spiking() {
        return Math.abs(Pigeon.getChangePerSecond().pitchPerSecond) > 10;
    }

    /**
     * Print information about auto balance.
     */
    public static void print() {
        SmartDashboard.putString("Auto Balance Stage: ", instance.stage.toString());
    }
}
