package frc.robot.systems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SmartPrintable;

/**
 * Manages the SwerDrive class for auto balancing, this is only seperate since
 * SwerveDrive was becoming a bit large to handle.
 */
public class AutoBalance extends SmartPrintable {
    private static final double RAMMING_SPEED = -0.65;
    private static final double HALTING_SPEED = 0.1;
    
    private static final AutoBalance instance = new AutoBalance();

    private Stage stage;
    private BalanceType type;
    private boolean rammingSubStageTwo;
    private Timer timer;

    public enum BalanceType {
        OVER_AND_BACK,
        DOCK
    }

    public enum Stage {
        /** 
         * auto balance is doing nothing. 
         */
        IDLING,

        /** 
         * auto balance is ramming, meant to dock the charge station. 
         */

        RAMMING,

        /** 
         * auto balance is on the charge station and trying to engage the charge
         * station. 
         */
        ADJUSTING,
        
        /** 
         * auto balance has paused, either becaue it has finished, or because 
         * it has encountered a spike in angular change. 
         */
        PAUSED,
    }

    private AutoBalance() {
        super();
        stage = Stage.IDLING;
        type = BalanceType.DOCK;
        rammingSubStageTwo = false;
        timer = new Timer();
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
                instance.stage = instance.ram();
                break;  
            
            case ADJUSTING:
                instance.stage = instance.adjust();
                break;
            
            case PAUSED:
                instance.stage = instance.pause();
                break;
            
            default:
                throw new RuntimeException("AutoBalance stage was misshandled. Value: " + instance.stage.toString());
        }

        instance.print();
    }

    /**
     * Gets the auto balance stage.
     */
    public static Stage getStage() {
        return instance.stage;
    }

    public static BalanceType getType() {
        return instance.type;
    }

    /**
     * Manually sets the stage of auto balance.
     */
    public static void setStage(Stage stage) {
        instance.stage = stage;

        if (stage == Stage.IDLING) {
            instance.rammingSubStageTwo = false;
        }
    }

    public static void setType(BalanceType type) {
        instance.type = type;
    }

    private Stage ram() {
        if (Math.abs(SwerveDrive.getDistance()) > 330 && type == BalanceType.DOCK) {
            return Stage.ADJUSTING;
        } else if (Math.abs(SwerveDrive.getDistance()) > 330 + 330 + 30 && !rammingSubStageTwo) {
            rammingSubStageTwo = true;
            timer.reset();
            timer.start();
            SwerveDrive.zeroPositions();
        } else if (Math.abs(SwerveDrive.getDistance()) > 330 + 60 + 90 && rammingSubStageTwo && timer.get() > 1.0) {
            return Stage.ADJUSTING;
        }

        SwerveDrive.runUncurved(
            0.0, 
            RAMMING_SPEED 
                * (rammingSubStageTwo ? -1.0 : 1.0)
                * (Math.abs(SwerveDrive.getDistance()) > 330 ? 0.5 : 1.0), 
            0.0
        );

        return Stage.RAMMING;
    }

    private Stage adjust() {
        if (level() || spiking()) {
            return Stage.PAUSED;
        }

        SwerveDrive.runUncurved(0.0, Math.signum(Pigeon.getPitch()) * HALTING_SPEED, 0.0);
        return Stage.ADJUSTING;
    }

    private Stage pause() {
        if (!level() && !spiking()) {
            return Stage.ADJUSTING;
        }

        //SwerveDrive.setRockMode(true);
        SwerveDrive.runUncurved(0.0, 0.0, 0.0);
        //SwerveDrive.runRockMode();
        return Stage.PAUSED;
    }

    /**
     * Returns true if auto balance beleives it is balanced, not 100% accurate.
     */
    private boolean level() {
        return Math.abs(Pigeon.getPitch()) < 10.0 && Math.abs(Pigeon.getChangePerSecond().pitchPerSecond) < 10.0;
    }

    /**
     * Returns true if auto balance beleives it is spiking in angle.
     */
    private boolean spiking() {
        return Math.abs(Pigeon.getChangePerSecond().pitchPerSecond) > 10.0;
    }

    /**
     * Print information about auto balance.
     */
    @Override
    public void print() {
        SmartDashboard.putString("Auto Balance Stage: ", instance.stage.toString());
        SmartDashboard.putBoolean("Auto Balance Spiking ", spiking());
        SmartDashboard.putBoolean("Auto Balance Balanced ", level());
    }
}
