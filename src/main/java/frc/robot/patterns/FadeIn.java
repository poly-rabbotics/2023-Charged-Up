package frc.robot.patterns;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LightPattern;

/**
 * Swell into a solid color.
 */
public class FadeIn implements LightPattern {
	Color[] pattern;
	final Color color;
	double speed;
	double time = 0.0;
	boolean fadedIn = false;

	public FadeIn() {
		speed = 0.4;
		color = new Color(0.0, 1.0, 0.0);
		pattern = new Color[] { color };
	}

	/**
	 * Creates a new {@link Breathe} instance from the given color at the given speed.
	 *
	 * @param color
	 * The color to breathe with.
	 *
	 * @param speed
	 * The speed to breathe at, should be lower than most other patterns.
	 */
	public FadeIn(Color color, double speed) {
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
		return fadedIn;
	}

	@Override
	public boolean isEqual(LightPattern pattern) {
		if (pattern.getClass() != this.getClass()) {
			return false;
		}

		FadeIn castPattern = (FadeIn)pattern;
		return castPattern.color.red == color.red && 
			   castPattern.color.green == color.green &&
			   castPattern.color.blue == color.blue &&
			   castPattern.speed == speed;
	}

	private void updatePattern() {
		if (fadedIn) {
			pattern[0] = color;
			return;
		}

		double fadeAmount = Math.pow(speed * time, 3);
		fadedIn = fadeAmount >= 1.0;
		pattern[0] = new Color(color.red * fadeAmount, color.green * fadeAmount, color.blue * fadeAmount);
	}
}
