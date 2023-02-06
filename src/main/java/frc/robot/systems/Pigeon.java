// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems;

import java.time.Clock;
import java.time.Instant;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.sensors.Pigeon2;

public class Pigeon {
    private static final int PIGEON_CAN_ID = 9;

    private static Pigeon instance = new Pigeon(PIGEON_CAN_ID);

    private Pigeon2 pigeon;
    private ScheduledExecutorService changeRateThread;
    private double relativeForward = 0.0;

    private OrientationalChange changePerSecond;

    private Pigeon(int canID) {
        pigeon = new Pigeon2(canID);

        /* 
         * Starts the thread so that it calls 'run()' every 40 ms (25hz). This
         * automatically updates the changePerSecond* feilds at that rate.
         */ 
        OrientationalChangeCalculator angularChangeCalculator = new OrientationalChangeCalculator(this);
        changeRateThread = Executors.newSingleThreadScheduledExecutor();
        changeRateThread.scheduleAtFixedRate(angularChangeCalculator, 0, 40, TimeUnit.MILLISECONDS);
    }

    /**
     * Sets "relative forward" to the current position. This will make getting
     * the relative rotation always return a rotation relative to the rotation
     * the robot is in at the point this method is called.
     */
    public static void setRelativeForward() {
        instance.relativeForward = getAbsoluteRotationDegrees();
    }

    /**
     * Gets the robot's rotation in respect to relative forward, if relative
     * forward has not been set then it simply returns the absolute rotaton.
     */
    public static double getRelativeRotationDegrees() {
        return (getAbsoluteRotationDegrees() - instance.relativeForward) % 360.0;
    }

    /**
     * Gets the absolute rotation of the Pigeon.
     */
    public static double getAbsoluteRotationDegrees() {
        return instance.pigeon.getYaw() % 360.0;
    }

    /**
     * Gets the change per second in the orientation of the Pigeon.
     */
    public static OrientationalChange getChangePerSecond() {
        return instance.changePerSecond;
    }

    /**
     * Represents the change per second in orientation as gathered and 
     * calculated from the Pigeon.
     */
    public static class OrientationalChange {
        public final double yaw;
        public final double roll;
        public final double pitch;

        private OrientationalChange(double yaw, double roll, double pitch) {
            this.yaw = yaw;
            this.roll = roll;
            this.pitch = pitch;
        }
    }

    /*
     * This is made a subclass to make sure no one gets confused and tries to
     * run the whole Pigeon on a thread in Robot.java
     */
    private class OrientationalChangeCalculator implements Runnable {
        // This will only be a reference to the containing class.
        private Pigeon pigeon;

        private double previousYaw;
        private double previousRoll;
        private double previousPitch;

        // System clock and the instant in which the above doubles where recorded.
        private Clock clock;
        private Instant recordedInstant;

        private OrientationalChangeCalculator(Pigeon pigeon) {
            clock = Clock.systemDefaultZone();
            this.pigeon = pigeon;

            previousYaw = pigeon.pigeon.getYaw();
            previousRoll = pigeon.pigeon.getRoll();
            previousPitch = pigeon.pigeon.getPitch();

            recordedInstant = clock.instant();
        }

        @Override
        public void run() {
            Instant previousInstant = recordedInstant;

            double yaw = pigeon.pigeon.getYaw();
            double roll = pigeon.pigeon.getRoll();
            double pitch = pigeon.pigeon.getPitch();
            
            recordedInstant = clock.instant();
            
            double differenceSeconds = (double)(recordedInstant.toEpochMilli() - previousInstant.toEpochMilli()) * 1000.0;

            double changeYaw = (yaw - previousYaw) / differenceSeconds;
            double changeRoll = (roll - previousRoll) / differenceSeconds;
            double changePitch = (pitch - previousPitch) / differenceSeconds;

            pigeon.changePerSecond = new OrientationalChange(changeYaw, changeRoll, changePitch);
        }
    }
}
