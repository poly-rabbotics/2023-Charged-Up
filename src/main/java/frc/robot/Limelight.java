package frc.robot; 

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


//The "eyes" of the robot
public class Limelight implements Runnable {
	private static double CENTERING_TOLERANCE = 1.5; 
	private boolean isTracking = false;
	public boolean isCentered = false;

	private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
	private static final NetworkTableEntry pipeline = table.getEntry("pipeline");
	private static final NetworkTableEntry tx = table.getEntry("tx");
	private static final NetworkTableEntry ty = table.getEntry("ty");
	private static final NetworkTableEntry ta = table.getEntry("ta");
	private static final NetworkTableEntry tv = table.getEntry("tv");

	private double x, y, area, v;
	private int limelightProfileNum;
	private LimelightProfile limelightProfile;

    private int dial = 1; //UPDATE THESE FROM MECHANISMS JOYSTICK
    private int aprilTagDial = 8;

	//HOW TO CONNECT TO LIMELIGHT INTERFACE:
	//IN BROWSER, while connected to robot,
	//TRY limelight.local:5801

	private enum LimelightProfile {
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
	private Limelight() {}

	/**
	 * Gets the X position of this {@link Limelight}'s target.
	 */
	public static double getDegreesOffsetX() {
		return instance.x;
	}

	/**
	 * Gets the Y position of this {@link Limelight}'s target.
	 */
	public static double getDegreesOffsetY() {
		return instance.y;
	}

	/**
	 * Gets the area of this {@link Limelight}'s target.
	 */
	public static double getArea() {
		return instance.area;
	}

	/**
	 * @return true if limelight has a target in the current pipeline 
	 */
	public static boolean getTargetFound() {
		SmartDashboard.putBoolean("Tv?",table.containsKey("tv"));
		instance.v = tv.getDouble(0);
		return (instance.v == 0.0) ? false : true;
	}

	/**
	 * @return Current LimelightProfile 
	 */
	public static LimelightProfile getLimelightProfile() {
		return instance.limelightProfile;
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
	private void switchNetworkTablePipeline(int pipelineNumber) {
		pipeline.setNumber(pipelineNumber);
	}

	/**
     * Updates limelightProfile based on MechanismsJoystick.
     **/
	private void updateTrackingMode() {

        if(dial <= 2) {
			limelightProfileNum = dial;
		} else {
			limelightProfileNum = aprilTagDial;
		}

		switch (limelightProfileNum) {
			case 0:
				limelightProfile = LimelightProfile.CUBE;
				break;
			case 1:
				limelightProfile = LimelightProfile.CONE;
				break;
			case 2:
				limelightProfile = LimelightProfile.PEG;
				break;
			case 3:
				limelightProfile = LimelightProfile.ATAG1;
				break;
			case 4:
				limelightProfile = LimelightProfile.ATAG2;
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
		SmartDashboard.putNumber("Limelight Profile", limelightProfileNum);
	}

	// Updates feild based on network table data.
	private void retreiveNetworkTableData() {
		x = tx.getDouble(0.0);
		y = ty.getDouble(0.0);
		area = ta.getDouble(0.0);
		v = tv.getDouble(0.0);
	}

	/**
	 * Updates all feilds and properties of this {@link Limelight}.
	 */
	@Override
	public void run() {
		updateTrackingMode();
		retreiveNetworkTableData();

		SmartDashboard.putNumber("LimelightX", x);
		SmartDashboard.putNumber("LimelightY", y);
		SmartDashboard.putNumber("LimelightArea", area);
		SmartDashboard.putBoolean("Target centered?", isCentered);

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
	}

	/* private void trackingMode() {
		camMode.setDouble(0);
		ledMode.setDouble(0);
		isTracking = true;
	} */

	/* private void calibrateLimelight(){
		//trackingMode();
		//read values periodically
		isTracking = true;
		x = tx.getDouble(0.0);
		y = ty.getDouble(0.0);
		area = ta.getDouble(0.0);
		ledMode = table.getEntry("ledMode");
		camMode = table.getEntry("camMode");    
		//post to smart dashboard periodically
		SmartDashboard.putNumber("LimelightX", x);
		SmartDashboard.putNumber("LimelightY", y);
		SmartDashboard.putNumber("LimelightArea", area);
		if (getTargetFound() && isTracking) {
			if (x < -CENTERING_TOLLERANCE || x > CENTERING_TOLLERANCE) {
				SmartDashboard.putBoolean("is centered", false);
			} else {
				SmartDashboard.putBoolean("is centered", true);
			}
		} else {
			trackingMode();
			isTracking = false;
			SmartDashboard.putBoolean("is centered", false);
		}    
	} */
}
