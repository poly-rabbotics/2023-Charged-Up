package frc.robot.patterns;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.helperClasses.LightPattern;

/**
 * A pattern that creates a swell up and down effect using brightness.
 */
public class Breathe implements LightPattern {
	Color[] pattern;
	Color color;
	double speed;
	double time = 0.0;
	boolean requestingReset = false;

	/**
	 * Creates a new {@link Breathe} instance from the given color at the given speed.
	 *
	 * @param color
	 * The color to breathe with.
	 *
	 * @param speed
	 * The speed to breathe at, should be lower than most other patterns.
	 */
	public Breathe(Color color, double speed) {
		this.speed = speed;
		this.color = color;
		pattern = new Color[] { color };
	}

	@Override
	public Color[] getPattern(double time) {
		this.time = time;
		updatePattern();
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

	private void updatePattern() {
		double fadeAmount = speed * time;
		requestingReset = fadeAmount >= 1.0;
		
		// If we get halfway through our timer period we want to fade back down.
		if (fadeAmount > 0.5)
			fadeAmount = 0.5 - (fadeAmount - 0.5);
		fadeAmount *= 2;

		pattern[0] = new Color(color.red * fadeAmount, color.green * fadeAmount, color.blue * fadeAmount);
	}
}
