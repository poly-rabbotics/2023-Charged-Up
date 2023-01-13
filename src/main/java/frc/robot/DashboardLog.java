package frc.robot;

import java.util.Date;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A class for logging errors and warnings to the smart dashboard that automatically adds timestamps.
 */
public class DashboardLog {
	private Date time = new Date();

	private String warningsLog = "";
	private String warningsDetails = "";

	private String errorsLog = "";
	private String errorsDetails = "";

    private static DashboardLog instance = new DashboardLog();

	/**
	 * Logs an error.
	 * 
	 * @param error
	 * The message to log.
	 */
	public static void logError(String error) {
		instance.errorsLog += instance.getCurrentTime() + error + "\n";
		instance.putErrors();
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
		instance.errorsLog += instance.getCurrentTime() + error + "\n";
		instance.errorsDetails += instance.getCurrentTime() + exception.getMessage() + "\n";
		instance.putErrors();
	}

	/**
	 * Logs a warning message.
	 * 
	 * @param warning
	 * The message to log.
	 */
	public static void logWarning(String warning) {
        instance.warningsLog += warning + "\n";
		instance.putWarnings();
	}

	/**
	 * Logs an exception and its details.
	 * 
	 * @param exception
	 * The exception to log.
	 */
	public static void logWarning(Exception exception) {
		// In the event that the class is anonymous it returns null.
		String warning = (exception.getClass().getCanonicalName() != null) ? exception.getClass().getCanonicalName() : "Exception in anonymous or unknown class.";
		instance.warningsLog += instance.getCurrentTime() + warning + "\n";
		instance.warningsDetails += instance.getCurrentTime() + exception.getMessage() + "\n";
		instance.putWarnings();
	}

	// Puts errors to smart dashboard.
	private void putErrors() {
		SmartDashboard.putString("Error Readouts", errorsLog);
		SmartDashboard.putString("Error Readouts Detailed", errorsDetails);
	}

	// Puts warnings to smart dashboard.
	private void putWarnings() {
		SmartDashboard.putString("Warning Readouts", warningsLog);
		SmartDashboard.putString("Warning Readouts Detailed", warningsDetails);
	}

	// Returns a prefixe for all log messages that includes a timestamp.
	private String getCurrentTime() {
		// Cuts the dates time string so that only hh:mm:ss is left and formats to prefix errors.
		return "[" + time.toString().substring(12, 21) + "] : ";
	}

	private DashboardLog() {
		errorsLog = getCurrentTime() + "Started error log";
		errorsDetails = getCurrentTime() + "Started error log";
		warningsLog = getCurrentTime() + "Started warning log";
		warningsDetails = getCurrentTime() + "Started warning log";
	}
}
