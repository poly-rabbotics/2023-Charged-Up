// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.TooManyListenersException;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SmartPrintable;

public class AxisRateLimiter extends SmartPrintable {
    private String id;
    private Timer timer = new Timer();

    private double changePerSec;
    private double previousValue = 0.0;

    private boolean enabled = true;

    public AxisRateLimiter(double changePerSec, String id) {
        this.id = id;
        this.changePerSec = changePerSec;

        timer.reset();
        timer.start();
    }

    public double apply(double axis) {
        if (!enabled) {
            previousValue = axis;

            timer.reset();
            timer.start();

            return axis;
        }

        double difference = Math.abs(previousValue - axis);
        double speed = difference * timer.get();

        if (speed > changePerSec) {
            axis += (timer.get() * changePerSec * Math.signum(axis));
        }

        previousValue = axis;

        timer.reset();
        timer.start();
        
        return axis;
    }

    public String getId() {
        return id;
    }

    public void setId(String id) {
        this.id = id;
    }

    public double getChangePerSec() {
        return changePerSec;
    }

    public void setChangePerSec(double changePerSec) {
        this.changePerSec = changePerSec;
    }

    public void toggleEnabled() {
        timer.reset();
        timer.start();

        enabled = !enabled;
    }

    public void setEnabled(boolean enabled) {
        timer.reset();
        timer.start();
        
        this.enabled = enabled;
    }

    @Override
    public void print() {
        SmartDashboard.putNumber("Axis Rate Limiter (" + id + ") Value", previousValue);
    }
}
