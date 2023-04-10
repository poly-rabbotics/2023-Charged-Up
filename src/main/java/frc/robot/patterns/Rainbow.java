package frc.robot.patterns;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.LightPattern;
import frc.robot.systems.LEDLights;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Rainbow implements LightPattern {
	private static final int COLOR_SATURATION = 255;
	private static final int COLOR_BRIGHTNESS = 128;

	private boolean requestingReset = false;
	private double time = 0.0;
	private double speed;
	private Color[] pattern;

	@Override
	public Color[] getPattern(double time) {

		if (requestingReset) {
			final String message = "Timer reset request was not reciprocated. \"requestingReset\" was not of expected value \"false\".\nValue was:";
			SmartDashboard.putBoolean(message, requestingReset);
		}
		
		if (time == this.time)
			return pattern;

		this.time = time;
		updatePattern();
		return pattern;
	}

	@Override
	public boolean getShouldResetTimer() {
		return requestingReset;
	}

	@Override
	public int getPatternLength() {
		return pattern.length;
	}
	
	@Override
	public boolean isEqual(LightPattern pattern) {
		if (pattern == null) {
			return false;
		}
		
		if (pattern.getClass() != this.getClass()) {
			return false;
		}

		Rainbow castPattern = (Rainbow)pattern;

		return speed == castPattern.speed;
	}

	private void updatePattern() {		
		final double hueShift = time * speed;

		for (var i = 0; i < pattern.length; i++) {
            final double hue = (hueShift + (i * 180 / pattern.length)) % 180;
            pattern[i] = Color.fromHSV((int)hue, COLOR_SATURATION, COLOR_BRIGHTNESS);
        }

		requestingReset = hueShift >= 180.0;
	}

	public Rainbow() {
		this.speed = 50;
		pattern = new Color[LEDLights.LED_LENGTH];
		updatePattern();
	}

	/**
	 * EPIC GAMER LIGHTS GO BRRRRR
	 * @param length
	 * @param speed
	 */
	public Rainbow(int length, double speed) {
		this.speed = speed;
		pattern = new Color[length];
		updatePattern();
	}
}
