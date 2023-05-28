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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SmartPrintable;

/*
 * Manages the robot's pigeon.
 */
public class Pigeon extends SmartPrintable {
    private static final int PIGEON_CAN_ID = 20;
    private static final Pigeon instance = new Pigeon(PIGEON_CAN_ID);

    private final Pigeon2 pigeon;
    private final ScheduledExecutorService changeRateThread;
    
    private OrientationalChange changePerSecond;

    private Pigeon(int canID) {
        super();
        pigeon = new Pigeon2(canID);

        /* 
         * Starts the thread so that it calls 'run()' every 40 ms (25hz). This
         * automatically updates the changePerSecond feilds at that rate.
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
    public static void setFeildZero() {
        instance.pigeon.setYaw(0.0);
    }

    /**
     * Gets the change per second in the orientation of the Pigeon.
     */
    public static OrientationalChange getChangePerSecond() {
        return instance.changePerSecond;
    }

    /**
     * Gets Yaw.
     */
    public static double getYaw() {
        return instance.pigeon.getYaw() % 360.0;
    }

    /**
     * Gets pitch.
     */
    public static double getPitch() {
        // Pitch and roll are flipped due to mounting orientation.
        return instance.pigeon.getRoll();
    }

    /**
     * Gets roll.
     */
    public static double getRoll() {
        // Pitch and roll are flipped due to mounting orientation.
        return instance.pigeon.getPitch();
    }
    
    @Override
    public void print() {
        SmartDashboard.putNumber("Pigeon Yaw", getYaw());
        SmartDashboard.putNumber("Pigeon Pitch", getPitch());
        SmartDashboard.putNumber("Pigeon Roll", getRoll());

        OrientationalChange change = getChangePerSecond();

        SmartDashboard.putNumber("Pigeon Yaw/Sec", change.yawPerSecond);
        SmartDashboard.putNumber("Pigeon Pitch/Sec", change.pitchPerSecond);
        SmartDashboard.putNumber("Pigeon Roll/Sec", change.rollPerSecond);
    }

    /**
     * Represents the change per second in orientation as gathered and 
     * calculated from the Pigeon.
     */
    public static class OrientationalChange {
        public final double yawPerSecond;
        public final double rollPerSecond;
        public final double pitchPerSecond;

        private OrientationalChange(double yaw, double roll, double pitch) {
            this.yawPerSecond = yaw;
            this.rollPerSecond = roll;
            this.pitchPerSecond = pitch;
        }
    }

    /*
     * This is made a subclass to make sure no one gets confused and tries to
     * run the whole Pigeon on a thread in Robot.java
     */
    private class OrientationalChangeCalculator implements Runnable {
        private final Pigeon pigeon;  // Reference to containing pigeon.
        private final Clock clock;    // Clock used to get current instant.

        private Instant recordedInstant;
        
        private double previousYaw;
        private double previousRoll;
        private double previousPitch;

        private OrientationalChangeCalculator(Pigeon pigeon) {
            this.pigeon = pigeon;
            clock = Clock.systemDefaultZone();
            recordedInstant = clock.instant();
            
            previousYaw = pigeon.pigeon.getYaw();
            previousRoll = pigeon.pigeon.getPitch(); // Roll and Pitch are swapped cause of the way its mounted.
            previousPitch = pigeon.pigeon.getRoll();
        }

        @Override
        public void run() {
            Instant previousInstant = recordedInstant;

            double yaw = pigeon.pigeon.getYaw();
            double roll = pigeon.pigeon.getPitch(); // Roll and Pitch are swapped cause of the way its mounted.
            double pitch = -pigeon.pigeon.getRoll(); // Negative since it should be positive going up.
            
            recordedInstant = clock.instant();
            
            double differenceSeconds = (double)(recordedInstant.toEpochMilli() - previousInstant.toEpochMilli()) / 1000.0;

            double changeYaw = (yaw - previousYaw) / differenceSeconds;
            double changeRoll = (roll - previousRoll) / differenceSeconds;
            double changePitch = -((pitch - previousPitch) / differenceSeconds);

            pigeon.changePerSecond = new OrientationalChange(changeYaw, changeRoll, changePitch);

            previousYaw = yaw;
            previousRoll = roll;
            previousPitch = pitch;
        }
    }
}
