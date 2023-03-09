package frc.robot.patterns;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LightPattern;

public class Flashing implements LightPattern{

    public double speed;
    public double time = 0.00;
    public Color[] pattern;

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

    private void updatePattern(){
        
    }
    
    public Flashing(double speed,  Color[] pattern){
        this.speed = speed;
        this.pattern = pattern;
        updatePattern();
    }
}
