package frc.robot;

import java.util.Date;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A class for logging errors and warnings to the smart dashboard that automatically adds timestamps.
 */
public class DashboardLog {
	private static Date time = new Date();

	private String warningsLog = "";
	private String warningsDetails = "";

	private static String errorsLog = "";
	private static String errorsDetails = "";

	/**
	 * Logs an error.
	 * 
	 * @param error
	 * The message to log.
	 */
	public void logError(String error) {
		errorsLog += getCurrentTime() + error + "\n";
		putErrors();
	}

	/**
	 * Logs an exception and its details.
	 * 
	 * @param exception
	 * The exception to log.
	 */
	public static void logError(Exception exception) {
		// In the event that the class is anonymous it returns null.
		String error = (exception.getClass().getCanonicalName() != null) ? exception.getClass().getCanonicalName() : "Exception in anonymous or unknown class.";
		errorsLog += getCurrentTime() + error + "\n";
		errorsDetails += getCurrentTime() + exception.getMessage() + "\n";
		putErrors();
	}

	/**
	 * Logs a warning message.
	 * 
	 * @param warning
	 * The message to log.
	 */
	public void logWarning(String warning) {
		warningsLog += warning + "\n";
		putWarnings();
	}

	/**
	 * Logs an exception and its details.
	 * 
	 * @param exception
	 * The exception to log.
	 */
	public void logWarning(Exception exception) {
		// In the event that the class is anonymous it returns null.
		String warning = (exception.getClass().getCanonicalName() != null) ? exception.getClass().getCanonicalName() : "Exception in anonymous or unknown class.";
		warningsLog += getCurrentTime() + warning + "\n";
		warningsDetails += getCurrentTime() + exception.getMessage() + "\n";
		putWarnings();
	}

	// Puts errors to smart dashboard.
	private static void putErrors() {
		SmartDashboard.putString("Error Readouts", errorsLog);
		SmartDashboard.putString("Error Readouts Detailed", errorsDetails);
	}

	// Puts warnings to smart dashboard.
	private void putWarnings() {
		SmartDashboard.putString("Warning Readouts", warningsLog);
		SmartDashboard.putString("Warning Readouts Detailed", warningsDetails);
	}

	// Returns a prefixe for all log messages that includes a timestamp.
	private static String getCurrentTime() {
		// Cuts the dates time string so that only hh:mm:ss is left and formats to prefix errors.
		return "[" + time.toString().substring(12, 21) + "] : ";
	}

	public DashboardLog() {
		errorsLog = getCurrentTime() + "Started error log";
		errorsDetails = getCurrentTime() + "Started error log";
		warningsLog = getCurrentTime() + "Started warning log";
		warningsDetails = getCurrentTime() + "Started warning log";
	}

	public static DashboardLog instance = new DashboardLog();
}
