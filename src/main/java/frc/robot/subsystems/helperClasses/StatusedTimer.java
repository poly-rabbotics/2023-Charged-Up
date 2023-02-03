package frc.robot.subsystems.helperClasses;

import edu.wpi.first.wpilibj.Timer;

/**
 * A class extending {@link Timer} to track its current running status.
 */
public class StatusedTimer extends Timer {
	private boolean running = false;

	/**
	 * Gets the status of this {@link StatusedTimer}.
	 * 
	 * @return
	 * <code>true</code> if the timer is running. <code>false</code> if the timer is stopped. This is not affected by the 
	 * status of any <code>wait()</code> calls. 
	 */
	public boolean getStatus() {
		return running;
	}

	@Override
	public void start() {
		running = true;
		super.start();
	}

	@Override
	public void stop() {
		running = false;
		super.stop();
	}
}
