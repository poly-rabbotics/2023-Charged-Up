package frc.robot.patterns;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LightPattern;
import frc.robot.systems.SwerveDrive;

public class BensPattern implements LightPattern {


    private Color color;
    private double speed;
    private boolean requestingReset;

    public BensPattern(Color color, double speed){
        this.color = color;
        this.speed = speed;
    }

    @Override
    public Color[] getPattern(double time) {
        if (time * speed < 0.5) {
            requestingReset = false;
            return new Color[] { new Color(0,0,0) };
        } else if(time * speed < 1) {
            requestingReset = false;
            return new Color[] { color };
        } else {
            requestingReset = true;
            return new Color[] { new Color(0,0,0) };
        }
    }

    @Override
    public boolean getShouldResetTimer() {
        return requestingReset;

    }

    @Override
    public int getPatternLength() {
        return 1;
    }

    @Override
    public boolean isEqual(LightPattern pattern) {
        return false;
    }
    
}
