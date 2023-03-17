package frc.robot; 

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


//The "eyes" of the robot
public class Limelight{
	private static double CENTERING_TOLERANCE = 1.5; 
	private static boolean isTracking = false;
	public static boolean isCentered = false;

	public static boolean moveRight, moveLeft;

	private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
	private static final NetworkTableEntry pipeline = table.getEntry("pipeline");
	private static final NetworkTableEntry tx = table.getEntry("tx");
	private static final NetworkTableEntry ty = table.getEntry("ty");
	private static final NetworkTableEntry ta = table.getEntry("ta");
	private static final NetworkTableEntry tv = table.getEntry("tv");

	private static double x;
	private static double y;
	private static double area;
	private static double v;
	private static int limelightProfileNum;
	private static LimelightProfile limelightProfile;
	private static String limelightProfileName;

	//HOW TO CONNECT TO LIMELIGHT INTERFACE:
	//IN BROWSER, while connected to robot,
	//TRY limelight.local:5801

	private enum LimelightProfile { //TODO: calibrate these different pipelines
		CUBE,
		CONE,
		PEG,
		ATAG1, ATAG2, ATAG3, ATAG4, ATAG5, ATAG6, ATAG7, ATAG8
	}

	public static Limelight instance = new Limelight();

	/**
	 * Creates a new instance of {@link Limelight}. There may only be one instance of this class and more than one warrants
	 * undocumented behavior or failure.
	 */
	private Limelight() {
		//executorService = Executors.newSingleThreadScheduledExecutor();
		//executorService.scheduleAtFixedRate(instance, 0, 20, TimeUnit.MILLISECONDS);
	}

	/**
	 * Gets the X position of this {@link Limelight}'s target.
	 */
	public static double getDegreesOffsetX() {
		return x;
	}

	/**
	 * Gets the Y position of this {@link Limelight}'s target.
	 */
	public static double getDegreesOffsetY() {
		return y;
	}

	/**
	 * Gets the area of this {@link Limelight}'s target.
	 */
	public static double getArea() {
		return area;
	}

	/**
	 * @return true if limelight has a target in the current pipeline 
	 */
	public static boolean getTargetFound() {
		SmartDashboard.putBoolean("Tv?",table.containsKey("tv"));
		v = tv.getDouble(0);
		return (v == 0.0) ? false : true;
	}

	/**
	 * @return Current LimelightProfile 
	 */
	public static LimelightProfile getLimelightProfile() {
		return limelightProfile;
	}

	/**
	 * Finishes executing the current <code>run()</code> call and terminates recursion.
	 */
	/* public void stopRunning() {
		threadRunning = false;
	} */

    /**
     * Switches pipline on the NetworkTable
     * @param pipelineNumber
     */
	private static void switchNetworkTablePipeline(int pipelineNumber) {
		pipeline.setNumber(pipelineNumber);
	}

	/**
     * Updates limelightProfile based on MechanismsJoystick.
     **/
	private static void updateTrackingMode(boolean button1Pressed, boolean button2Pressed, boolean button3Pressed, boolean button4Pressed) {
        if (button1Pressed) {
			limelightProfileNum = 0;
		} else if (button2Pressed) {
			limelightProfileNum = 1;
		} else if (button3Pressed) {
			limelightProfileNum = 2;
		} else if (button4Pressed) {
			limelightProfileNum = 3;
		}

		switch (limelightProfileNum) {
			case 0:
				limelightProfile = LimelightProfile.CUBE;
				limelightProfileName = "CUBE";
				break;
			case 1:
				limelightProfile = LimelightProfile.CONE;
				limelightProfileName = "CONE";
				break;
			case 2:
				limelightProfile = LimelightProfile.PEG;
				limelightProfileName = "PEG";
				break;
			case 3:
				limelightProfile = LimelightProfile.ATAG1;
				limelightProfileName = "APRIL TAG 1";
				break;
			case 4:
				limelightProfile = LimelightProfile.ATAG2;
				limelightProfileName = "APRIL TAG 2";
				break;
			case 5:
				limelightProfile = LimelightProfile.ATAG3;
				break;
			case 6:
				limelightProfile = LimelightProfile.ATAG4;
				break;
			case 7:
				limelightProfile = LimelightProfile.ATAG5;
				break;
			case 8:
				limelightProfile = LimelightProfile.ATAG6;
				break;
			case 9:
				limelightProfile = LimelightProfile.ATAG7;
				break;
			case 10:
				limelightProfile = LimelightProfile.ATAG8;
				break;
		}

		switchNetworkTablePipeline(limelightProfileNum);
	}

	// Updates feild based on network table data.
	private static void retreiveNetworkTableData() {
		x = tx.getDouble(0.0);
		y = ty.getDouble(0.0);
		area = ta.getDouble(0.0);
		v = tv.getDouble(0.0);
	}

	/**
	 * Updates all feilds and properties of this {@link Limelight}.
	 */
	
	public static void run(boolean button1Pressed, boolean button2Pressed, boolean button3Pressed, boolean button4Pressed) {
		updateTrackingMode(button1Pressed, button2Pressed, button3Pressed, button4Pressed);
		retreiveNetworkTableData();
		shuffleBoardOutput();
	}

	public static void shuffleBoardOutput() {
		SmartDashboard.putNumber("Limelight X Offset", x);
		SmartDashboard.putNumber("LimelightY", y);
		SmartDashboard.putNumber("LimelightArea", area);
		SmartDashboard.putNumber("Limelight Profile", limelightProfileNum);
		SmartDashboard.putBoolean("Target found?", getTargetFound());
		SmartDashboard.putString("Limelight Profile Name", limelightProfileName);
		if (x > CENTERING_TOLERANCE) {
			moveRight = true;
			isCentered = false;
			moveLeft = false;
		}
		if (x < -CENTERING_TOLERANCE) {
			moveLeft = false;
			isCentered = false;
			moveRight = false;
		}

		if (getTargetFound() && isTracking) {
			if (x < -CENTERING_TOLERANCE || x > CENTERING_TOLERANCE) {
				isCentered = false;
			} else {
				isCentered = true;
			}
		} else {
			isTracking = false;
			isCentered = false;
		}

		//ORGANIZE THESE AS VISUAL DRIVER AIDS: LEFT , CENTER, RIGHT
		//AND WHICHEVER IS GREEN WE WILL DRIVE THAT WAY UNTIL CENTERED
		SmartDashboard.putBoolean("Move Right?", moveRight);
		SmartDashboard.putBoolean("Target centered?", isCentered);
		SmartDashboard.putBoolean("Move Left?", moveLeft);

	}

}
