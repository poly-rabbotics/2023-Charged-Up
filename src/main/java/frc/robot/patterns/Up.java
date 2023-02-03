package frc.robot.patterns;

import frc.robot.subsystems.helperClasses.LightPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

/**
 * A class for producing a solid color on light strips.
 */
public class Up implements LightPattern {
    Color[] pattern;
    int[] rainbowHueArr;
    int length;
    double time;
    double speed;
    double rainbowSpeed;
    int hue = 68;
    int trailLength;
    boolean requestingReset = false;
    boolean rainbowMode = false;

    /**
     * Creates a new {@link Up} pattern.
     *
     * @param r
     *          This {@link Up}'s red component.
     *
     * @param g
     *          This {@link Up}'s green component.
     *
     * @param b
     *          This {@link Up}'s blue component.
     */
    public Up(double speed, int length, int trailLength, boolean rainbowMode) {
        pattern = new Color[length];
        rainbowHueArr = new int[length];
        this.speed = speed;
        this.length = length;
        this.trailLength = trailLength;
        this.rainbowMode = rainbowMode;
    }

    public Up(double speed, int length, int hue, int trailLength) {
        pattern = new Color[length];
        this.speed = speed;
        this.length = length;
        this.hue = hue;
        this.trailLength = trailLength;
    }

    private void updatePattern() {
        int position = (int) ((time * speed) % pattern.length);
        int increment = 255 / trailLength;

        if (position >= pattern.length)
            requestingReset = true;
        else
            requestingReset = false;

        for (int i = 0; i < pattern.length; i++) {
            pattern[i] = Color.fromHSV(0, 0, 0);
            rainbowHueArr[i] = (int) ((180 / pattern.length) * i);
        }

        for (int i = 0; i < pattern.length; i++) {
            if (i + 1 < pattern.length)
                rainbowHueArr[i] = rainbowHueArr[i + 1];
            else
                rainbowHueArr[i] = rainbowHueArr[0];

            if (i == position) {
                for (int j = 0; j < trailLength; j++) {

                    if (i - j >= 0) {
                        if (rainbowMode)
                            hue = rainbowHueArr[i - j];
                        pattern[i - j] = Color.fromHSV(hue, 255, 255 - (increment * j));
                    } else {
                        if (rainbowMode)
                            hue = rainbowHueArr[pattern.length + (i - j)];
                        pattern[pattern.length + (i - j)] = Color.fromHSV(hue, 255, 255 - (increment * j));
                    }
                }
                // increment += 255 / trailLength;
            }
        }

        SmartDashboard.putNumber("THING23", hue);

        /*
         * pattern[(int)position] = Color.fromHSV(hue, 255, 255);
         * for(int i = 0; i < trailLength; i++){
         * pattern[(int)position-i-1] = Color.fromHSV(hue, 255, (int)(255 -
         * (255-(255/trailLength)*(i+1))));
         * pattern[10] = Color.fromHSV(hue, 255, 255);
         * }
         */
    }

    @Override
    public Color[] getPattern(double time) {
        this.time = time;
        updatePattern();
        // requestingReset = position >= pattern.length + trailLength;
        return pattern;
    }

    @Override
    public int getPatternLength() {
        return pattern.length;
    }

    @Override
    public boolean getShouldResetTimer() {
        return requestingReset;
    }
}
