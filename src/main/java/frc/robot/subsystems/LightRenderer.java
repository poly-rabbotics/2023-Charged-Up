package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

import frc.robot.patterns.Rainbow;

/**
 * A class for rendering {@link LightPattern}s to lights strips on the robot.
 */
public class LightRenderer implements Runnable {
	private static final double DEFAULT_RAINBOW_SPEED = 50.0;
	//public static final DigitalInput ledSwitch = new DigitalInput(0);

	private AddressableLED lightStrip;
	private AddressableLEDBuffer ledBuffer;
	private LightPattern[] patterns;
	private int[] patternBreaks;
	private StatusedTimer timer = new StatusedTimer();
	boolean changedPattern;
	int patternIndex = 0;

	public void setPatternBreaks(int[] patternBreaks) {
		this.patternBreaks = patternBreaks;
	}

	/**
	 * Sets the current light pattern and resets the internal timer.
	 */
	public void setPatterns(LightPattern[] pattern) {
		timer.reset();
		this.patterns = pattern;
	}

	public void setIfNotEqual(LightPattern[] patterns) {
		for (int i = 0; i < patterns.length; i++) {
			if (!this.patterns[i].isEqual(patterns[i])) {
				this.patterns[i] = patterns[i];
			}
		}

		timer.reset();
	}

	/**
	 * Stops this {@link LightRenderer}'s internal {@link StatusedTimer}.
	 */
	public void pause() {
		timer.stop();
	}

	/**
	 * Starts this {@link LightRenderer}'s internal {@link StatusedTimer}
	 */
	public void unpause() {
		timer.start();
	}

	/**
	 * Gets this {@link LightRenderer}'s internal {@link StatusedTimer}'s' status.
	 * 
	 * @return
	 * <code>true</code> if the timer is running. <code>false</code> if the timer is stopped.
	 */
	public boolean getPauseStatus() {
		return timer.getStatus();
	}

	/**
	 * Toggles this {@link LightRenderer}'s internal {@link StatusedTimer}'s' status.
	 * 
	 * @return
	 * <code>true</code> if the timer was started by this method. <code>false</code> if the timer was stopped.
	 */
	public boolean togglePauseStatus() {
		if (timer.getStatus())
			timer.stop();
		else
			timer.start();

		// Note that this timer.getStatus() call should always return the opposite value as was returned in the if statement
		// above, as it should have been toggled.
		return timer.getStatus();
	}

	/**
	 * Updates the LED light strip to account for time changes, calling this method continuously in a loop will allow patterns
	 * to appear fluid.
	 */
	public void run() {
		if (patternBreaks.length != patterns.length) {
			return;
		}

		int currentPattern = 0;

		for (int i = 0; i < patterns.length; i++) {
			Color[] colors = patterns[i].getPattern(timer.get());
			Color color = colors[i % patterns[i].getPatternLength()];
			ledBuffer.setLED(i, color);

			if (patternBreaks[currentPattern] == i) {
				currentPattern++;
			}
		}

		for (LightPattern pattern : patterns) {
			if (pattern.getShouldResetTimer()) {
				timer.reset();
			}
		}

		lightStrip.setData(ledBuffer);
		lightStrip.start();
	}

	/**
	 * Creates a new {@link LightPattern}.
	 * 
	 * @param addressableLEDPort
	 * The port number of {@link AddressableLED} that this {@link LightRenderer} should 
	 * use.
	 * 
	 * @param bufferLength
	 * The length of the light strip in pixels.
	 */
	public LightRenderer(int addressableLEDPort, int bufferLength) {
		lightStrip = new AddressableLED(addressableLEDPort);
		ledBuffer = new AddressableLEDBuffer(bufferLength);
		lightStrip.setLength(bufferLength);
		patterns[0] = new Rainbow(bufferLength, DEFAULT_RAINBOW_SPEED);
		patternBreaks[0] = bufferLength;
		lightStrip.start();
		timer.start();
	} 

	/**
	 * Creates a new {@link LightPattern}.
	 * 
	 * @param addressableLEDPort
	 * The port number of {@link AddressableLED} that this {@link LightRenderer} should 
	 * use.
	 * 
	 * @param bufferLength
	 * The length of the light strip in pixels.
	 * 
	 * @param pattern
	 * A {@link LightPattern} to be applied to following light renders in construction.
	 */
	public LightRenderer(int addressableLEDPort, int bufferLength, LightPattern[] patterns, int[] patternBreaks) {
		lightStrip = new AddressableLED(addressableLEDPort);
		ledBuffer = new AddressableLEDBuffer(bufferLength);
		lightStrip.setLength(bufferLength);
		this.patterns = patterns;
		this.patternBreaks = patternBreaks;
		timer.start();
	}
}
