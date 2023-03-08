package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color;

/**
 * Represents a repeating pattern of lights/colors using the {@link Color} class. This class keeps the number of colors in the
 * pattern immutable, so any classes using it can really on the pattern length to be unchanging, however the colors in the
 * pattern are free to change.
 */
public interface LightPattern {
	/**
	 * Gets the color in this {@link LightPattern} at the specified index.
	 * 
	 * @param index
	 * The index of the {@link Color} to get.
	 * 
	 * @param time
	 * The time from a {@link edu.wpi.first.wpilibj.Timer} in {@link LightRenderer} for dynamic patterns.
	 * 
	 * @return
	 * Returns a {@link Color}.
	 */
	public Color[] getPattern(double time);

	/**
	 * Returns true if the implementing class chould request a Timer.reset() call in {@link LightRenderer}.
	 */
	public boolean getShouldResetTimer();

	/**
	 * Gets this {@link LightPattern}s length.
	 */
	public int getPatternLength();

	/**
	 * Returns true if the given pattern is the same/equal.
	 */
	public boolean isEqual(LightPattern pattern);
}
