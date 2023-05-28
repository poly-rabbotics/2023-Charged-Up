// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

public final class Angle {
	private double dec = 0.0;

	public double radians() {
		return dec * Math.TAU;
	}

	public double degrees() {
		return dec * 360.0;
	}

	public Angle setRadians(double radians) {
		dec = radians / Math.TAU;
		return this;
	}

	public Angle setDegrees(double degrees) {
		dec = degrees / 360.0;
		return this;
	}

	/**
	 * Creates a new identical angle so that modification of the original will not
	 * effect the value returned from this function.
	 */
	public Angle clone() {
		var ang = new Angle();
		ang.dec = this.dec;
		return ang;
	}
}
