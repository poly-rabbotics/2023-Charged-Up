package frc.robot.subsystems.helperClasses;

import edu.wpi.first.wpilibj.util.Color;

public class ColorUtils {
	/**
	 * Creates a gradiant of {@link Color}s from two {@link Color}s.
	 * 
	 * <p> The light strip takes (255, 255, 255) to be the same as (1, 1, 1). 
	 * So this class has very little use. </p>
	 * 
	 * @param a
	 * The first color that should appear in the gradiant.
	 * 
	 * @param b
	 * The last color that should appear in the gradiant.
	 * 
	 * @param length
	 * The length of the gradiant incuding the two passed in colors. If ever thhis is less than 3, it will be treated as 
	 * though it was 3.
	 * 
	 * @return
	 * A {@link Color} array of the passed in length. 
	 */
	public static Color[] makeGradiant(Color a, Color b, int length) {
		if (length < 3) length = 3;
		
		Color[] colors = new Color[length];

		colors[0] = a;
		colors[length - 1] = b;

		double differenceR = b.red - a.red;
		double differenceG = b.green - a.green;
		double differenceB = b.blue - a.blue;

		for (int color = 1; color < length - 1; color++) {
			double incrementR = (differenceR / (length - 1) * color);
			double incrementG = (differenceG / (length - 1) * color);
			double incrementB = (differenceB / (length - 1) * color);

			colors[color] = new Color(a.red + incrementR, a.green + incrementG, a.blue + incrementB);
		}

		return colors;
	}
}
