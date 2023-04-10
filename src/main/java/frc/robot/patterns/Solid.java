// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.patterns;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LightPattern;

/** Add your docs here. */
public class Solid implements LightPattern {
    private Color color;

    public Solid(Color color) {
        this.color = color;
    }

    @Override
    public Color[] getPattern(double time) {
        return new Color[] { color };
    }

    @Override
    public boolean getShouldResetTimer() {
        return true;
    }

    @Override
    public int getPatternLength() {
        return 1;
    }

    @Override
    public boolean isEqual(LightPattern pattern) {
        if (pattern == null) {
            return false;
        }
        
        if (pattern.getClass() != this.getClass()) {
            return false;
        }

        return ((Solid)pattern).color == color;
    }
    
}
