package frc.robot; 

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;

//The "eyes" of the robot
public class Limelight implements Runnable {
	/* private static double CENTERING_TOLLERANCE = 1.5;
	private static final NetworkTableEntry tv = table.getEntry("tv");
	private NetworkTableEntry ledMode;
	private NetworkTableEntry camMode; 
	private boolean isTracking = false; */

	/* private static final double LOOP_INTERVAL = 0.010;
	private static final Timer timer = new Timer();
	private boolean threadRunning = false; */

	private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
	private static final NetworkTableEntry pipeline = table.getEntry("pipeline");
	private static final NetworkTableEntry tx = table.getEntry("tx");
	private static final NetworkTableEntry ty = table.getEntry("ty");
	private static final NetworkTableEntry ta = table.getEntry("ta");
	//public static final Servo servo = RobotMap.limelightServo;

	private double x, y, area;
	private int limelightProfile;

    private int dial;
    private int aprilTagDial;

	//HOW TO CONNECT TO LIMELIGHT INTERFACE:
	//IN BROWSER, while connected to robot,
	//TRY limelight.local:5801

	/**
	 * Creates a new instance of {@link Limelight}. There may only be one instance of this class and more than one warrants
	 * undocumented behavior or failure.
	 */
	public Limelight() { }

	/**
	 * Gets the X position of this {@link Limelight}'s target.
	 */
	public double getX() {
		return x;
	}

	/**
	 * Gets the Y position of this {@link Limelight}'s target.
	 */
	public double getY() {
		return y;
	}

	/**
	 * Gets the area of this {@link Limelight}'s target.
	 */
	public double getArea() {
		return area;
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
			limelightProfile = dial;
		} else {
			limelightProfile = aprilTagDial;
		}

		switchNetworkTablePipeline(limelightProfile);
		SmartDashboard.putNumber("Limelight Profile", limelightProfile);
	}

	// Updates servo's position based on limelightProfile.
	/*private void updateServoPosition() {
		
		if (limelightProfile == 2) {
			//REPLACE THESE SERVO VALUES FOR NEW SERVO
			servo.setAngle(180); 
		} else {
			servo.setAngle(70);
		} 
		SmartDashboard.putNumber("limelight servo", servo.getAngle());
	}*/

	// Updates feild based on network table data.
	private void retreiveNetworkTableData() {
		x = tx.getDouble(0.0);
		y = ty.getDouble(0.0);
		area = ta.getDouble(0.0);
	}

	/**
	 * Updates all feilds and properties of this {@link Limelight}.
	 * Note that this method blocks the current thread and loops, see <code>runNonBlocking()</code>.
	 */
	/* public void run() {
		while (threadRunning)
		{
			timer.reset();
			timer.start();
			// Run actual limelight code.
			runNonBlocking();
			// Waits for the specified interval to be reached.
			while (timer.get() < LOOP_INTERVAL) {
				try {
					// Wait a millisecond.
					timer.wait(1);
				} catch (InterruptedException e) { }
			}
		}
	} */

	/**
	 * Updates all feilds and properties of this {@link Limelight}.
	 */
	public void run() {
		updateTrackingMode();
		//updateServoPosition();
		retreiveNetworkTableData();

		SmartDashboard.putNumber("LimelightX", x);
		SmartDashboard.putNumber("LimelightY", y);
		SmartDashboard.putNumber("LimelightArea", area);
	}

	/* private boolean getTargetFound() {
		SmartDashboard.putBoolean("Tv?",table.containsKey("tv"));
		double v = tv.getDouble(0);
		if (v == 0.0) {
			return false;
		} else {
			return true;
		}
	} */

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
