// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.systems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SmartPrintable;

/**
 * Responsible for echolocation, hence Bat.
 */
public class Bat extends SmartPrintable {
    private static final int ECHO_CHANNEL = 2;
    private static final int PING_CHANNEL = 3;

    private static final double MAX_RANGE = 36.0;
    private static final double MAX_CHANGE_PER_SEC = 200.0;

    private static final Bat instance = new Bat();

    private final Ultrasonic ultrasonic;
    private final Timer timer;

    private double previousRange;
    private double previousChangePerSec;

    private Bat() {
        super();

        Ultrasonic.setAutomaticMode(true);
        ultrasonic = new Ultrasonic(PING_CHANNEL, ECHO_CHANNEL);
        timer = new Timer();

        timer.reset();
        timer.start();

        previousRange = 0.0;
        previousChangePerSec = 0.0;
    }

    public static boolean isInRange() {
        // NaN is not equal to itself :D.
        return instance.previousRange == instance.previousRange;
    }

    /**
     * Returns range in inches, -1.0 if out of range.
     */
    public static double getRange() {
        double range = instance.ultrasonic.getRangeInches();
        double frequency = 1.0 / instance.timer.get();
        double change = Math.abs(instance.previousRange - range);

        if (change * frequency > MAX_CHANGE_PER_SEC) {
            // Interpolate from the previous rate if we get a junk value.
            range = instance.previousRange + (instance.previousChangePerSec * frequency);
        }

        if (range > MAX_RANGE) {
            range = Double.NaN;
        }

        instance.previousRange = range;
        instance.previousChangePerSec = change;

        instance.timer.reset();
        instance.timer.start();
        
        return range;
    }

    @Override
    public void print() {
        SmartDashboard.putNumber("Bat Range", previousRange);
        SmartDashboard.putNumber("Bat Change/Sec", previousChangePerSec);
        SmartDashboard.putBoolean("Bat In Range", isInRange());
        SmartDashboard.putBoolean("Bat Wiring Error", previousRange == 0.0);
    }
}
