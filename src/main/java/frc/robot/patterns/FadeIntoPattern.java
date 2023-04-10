package frc.robot.patterns;

import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LightPattern;

/**
 * Swell into a solid color.
 */
public class FadeIntoPattern implements LightPattern {
	private LightPattern parentPattern;
    private double speed;
    private double time;
    private Color[] pattern;
    private boolean fadedIn;
    private Timer timer;

	/**
	 * Creates a new {@link Breathe} instance from the given color at the given speed.
	 *
	 * @param color
	 * The color to breathe with.
	 *
	 * @param speed
	 * The speed to breathe at, should be lower than most other patterns.
	 */
	public FadeIntoPattern(LightPattern pattern, double speed) {
		this.speed = speed;
		parentPattern = pattern;
        time = 0.0;
        this.pattern = parentPattern.getPattern(time);
        timer = new Timer();
        timer.reset();
        timer.start();
	}

	@Override
	public Color[] getPattern(double time) {
		this.time = time;
        pattern = parentPattern.getPattern(time);
		updatePattern();
		return pattern;
	}

	@Override
	public int getPatternLength() {
		return pattern.length;
	}

	@Override
	public boolean getShouldResetTimer() {
		return parentPattern.getShouldResetTimer();
	}

	@Override
	public boolean isEqual(LightPattern pattern) {
		if (pattern.getClass() != this.getClass()) {
			return false;
		}

		FadeIntoPattern castPattern = (FadeIntoPattern)pattern;
		return parentPattern.isEqual(castPattern.parentPattern) && speed == castPattern.speed;
	}

	private void updatePattern() {
		if (fadedIn) {
			pattern = parentPattern.getPattern(time);
			return;
		}

		double fadeAmount = Math.pow(speed * timer.get(), 3);
		fadedIn = fadeAmount >= 1.0;
		
        Color[] pattern = parentPattern.getPattern(time);

        for (int i = 0; i < pattern.length; i++) {
            pattern[i] = new Color(pattern[i].red * fadeAmount, pattern[i].green * fadeAmount, pattern[i].blue * fadeAmount);
        }
	}
}
