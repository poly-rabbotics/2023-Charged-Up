package frc.robot.patterns;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LightPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SolidColor implements LightPattern{

    private Color pattern;

    @Override
    public Color[] getPattern(double time) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public boolean getShouldResetTimer() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public int getPatternLength() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public boolean isEqual(LightPattern pattern) {
        // TODO Auto-generated method stub
        return false;
    }


    private SolidColor(Color pattern){
        this.pattern = pattern;
    }
    
}
