package frc.robot.systems;

import frc.robot.subsystems.Angle;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private static Limelight instance = new Limelight();

    private static final Angle LIMELIGHT_MOUNTING_PITCH = new Angle().setDegrees(0.0);
    private static final LimelightPos LIMELIGHT_MOUNTING_POS = instance.new LimelightPos(0.0, 0.0, 0.0); // inches
    private static final double APRIL_TAG_SIDE_LENGTH = 8.5; // inches

    // Red scoring target tags.
    private static final int APRIL_TAG_ID_RED_SPEAKER_CENTER = 4;
    private static final int APRIL_TAG_ID_RED_SPEAKER_OFFCENTER = 3;
    private static final int APRIL_TAG_ID_RED_AMP = 5;

    // Red stage tags. NOTE: Center stange faces middle of field, not drivers.
    private static final int APRIL_TAG_ID_RED_STAGE_CENTER = 13;
    private static final int APRIL_TAG_ID_RED_STAGE_RIGHT = 12;
    private static final int APRIL_TAG_ID_RED_STAGE_LEFT = 11;

    // Red intake tags. NOTE: Source is on opposite side of field.
    private static final int APRIL_TAG_ID_RED_SOURCE_DRIVER_SIDE = 9; 
    private static final int APRIL_TAG_ID_RED_SOURCE_FIELD_SIDE = 10;

    // Blue scoring target tags.
    private static final int APRIL_TAG_ID_BLUE_SPEAKER_CENTER = 7;
    private static final int APRIL_TAG_ID_BLUE_SPEAKER_OFFCENTER = 8;
    private static final int APRIL_TAG_ID_BLUE_AMP = 6;

    // Blue stage tags. NOTE: Center stange faces middle of field, not drivers.
    private static final int APRIL_TAG_ID_BLUE_STAGE_CENTER = 14;
    private static final int APRIL_TAG_ID_BLUE_STAGE_RIGHT = 16;
    private static final int APRIL_TAG_ID_BLUE_STAGE_LEFT = 15;

    // Blue intake tags. NOTE: Source is on opposite side of field.
    private static final int APRIL_TAG_ID_BLUE_SOURCE_DRIVER_SIDE = 1; 
    private static final int APRIL_TAG_ID_BLUE_SOURCE_FIELD_SIDE = 2;

    /**
     * Indexed by tag ID, given in inches to the center of the tag form field
     * floor.
     */
    private static final double[] APRIL_TAG_HEIGHTS = {
        // Feet + Inches + Fractional Inches + Half of April Tag Height
        (4.0 * 12.0) + (1.0 / 8.0) + (APRIL_TAG_SIDE_LENGTH / 2.0),        // 1
        (4.0 * 12.0) + (1.0 / 8.0) + (APRIL_TAG_SIDE_LENGTH / 2.0),        // 2
        (4.0 * 12.0) + 3.0 + (7.0 / 8.0) + (APRIL_TAG_SIDE_LENGTH / 2.0),  // 3
        (4.0 * 12.0) + 3.0 + (7.0 / 8.0) + (APRIL_TAG_SIDE_LENGTH / 2.0),  // 4
        (4.0 * 12.0) + (1.0 / 8.0) + (APRIL_TAG_SIDE_LENGTH / 2.0),        // 5
        (4.0 * 12.0) + (1.0 / 8.0) + (APRIL_TAG_SIDE_LENGTH / 2.0),        // 6
        (4.0 * 12.0) + 3.0 + (7.0 / 8.0) + (APRIL_TAG_SIDE_LENGTH / 2.0),  // 7
        (4.0 * 12.0) + 3.0 + (7.0 / 8.0) + (APRIL_TAG_SIDE_LENGTH / 2.0),  // 8
        (4.0 * 12.0) + (1.0 / 8.0) + (APRIL_TAG_SIDE_LENGTH / 2.0),        // 9
        (4.0 * 12.0) + (1.0 / 8.0) + (APRIL_TAG_SIDE_LENGTH / 2.0),        // 10
        (3.0 * 12.0) + 11.0 + (1.0 / 2.0) + (APRIL_TAG_SIDE_LENGTH / 2.0), // 11
        (3.0 * 12.0) + 11.0 + (1.0 / 2.0) + (APRIL_TAG_SIDE_LENGTH / 2.0), // 12
        (3.0 * 12.0) + 11.0 + (1.0 / 2.0) + (APRIL_TAG_SIDE_LENGTH / 2.0), // 13
        (3.0 * 12.0) + 11.0 + (1.0 / 2.0) + (APRIL_TAG_SIDE_LENGTH / 2.0), // 14
        (3.0 * 12.0) + 11.0 + (1.0 / 2.0) + (APRIL_TAG_SIDE_LENGTH / 2.0), // 15
        (3.0 * 12.0) + 11.0 + (1.0 / 2.0) + (APRIL_TAG_SIDE_LENGTH / 2.0), // 16  
    };


    public class LimelightOrPos {
        public LimelightPos pos;
        public LimelightOrientation orientation;

        public LimelightOrPos(LimelightPos pos, LimelightOrientation orientation) {
            this.pos = pos;
            this.orientation = orientation;
        }
    }

    /**
     * 3D position as calculated by the Limelight, may be relative to some other
     * point in space.
     */
    public class LimelightPos {
        public double x;
        public double y;
        public double z;

        public LimelightPos(double x, double y, double z) {
            this.x = x;
            this.y = y;
            this.z = z;
        }
    }

    /**
     * Orientation as calculated by the Limelight, may be the actual orientation
     * of an object, or may be the required re-orientation of the Limelight to 
     * center and object in the image.
     */
    public class LimelightOrientation {
        public Angle yaw;
        public Angle roll;
        public Angle pitch;

        public LimelightOrientation(Angle yaw, Angle roll, Angle pitch) {
            this.yaw = yaw;
            this.roll = roll;
            this.pitch = pitch;
        }
    }

    /**
     * Returns true if the Limelight has a valid target.
     */
    public static boolean hasValidTarget() {
        return tv() == 1;
    }

    /**
     * Returns the ID of the April Tag currently being targeted, or -1 if none
     * are being targeted.
     */
    public static int aprilTagTargetId() {
        if (!hasValidTarget()) {
            return -1;
        }

        double[] tid = tid();
        return (int)tid[0];
    }

    /**
     * Gets the target offset from the camera's pitch/vertical angle. Returns
     * `null` if no valid target exists.
     */
    public static Angle targetPitchOffset() {
        return hasValidTarget() 
            ? new Angle().setDegrees(ty()) 
            : null;
    }

    /**
     * Gets the target offset from the camera's yaw/horizontal angle. Returns
     * `null` if no valid target exists.
     */
    public static Angle targetYawOffset() {
        return hasValidTarget()
            ? new Angle().setDegrees(tx())
            : null;
    }

    /**
     * Gets the position of the Limelight relative to the blue side origin.
     */
    public static LimelightOrPos positionBlueSide() {
        double[] botpose = botpose_wpiblue();
        return doubleArrToOrPos(botpose);
    }

    /**
     * Gets the position of the Limelight relative to the red side origin.
     */
    public static LimelightOrPos positionRedSide() {
        double[] botpose = botpose_wpired();
        return doubleArrToOrPos(botpose);
    }

    /**
     * Gets the Limelight position relative to its target, returns null if there
     * is no valid target.
     */
    public static LimelightOrPos positionTargetSpace() {
        if (!hasValidTarget()) {
            return null;
        }
        
        double[] botpose = botpose_targetspace();
        return doubleArrToOrPos(botpose);
    }

    private static LimelightOrPos doubleArrToOrPos(double[] arr) {
        Angle yaw = new Angle().setDegrees(arr[5]);
        Angle roll = new Angle().setDegrees(arr[3]);
        Angle pitch = new Angle().setDegrees(arr[4]);
        
        LimelightPos pos = instance.new LimelightPos(arr[0], arr[1], arr[2]);
        LimelightOrientation ori = instance.new LimelightOrientation(yaw, roll, pitch);

        return instance.new LimelightOrPos(pos, ori);
    } 

    /**
     * Estimates the distance between the Limelight and its target. This 
     * distance is the distance as seen from a top down point of view. Returns
     * a value in inches or NaN on error.
     */
    private static double estimateTargetDistance() {
        int tagId = aprilTagTargetId();
        Angle targetVarticalAngleOffset = targetPitchOffset();

        if (tagId == -1) {
            return Double.NaN;
        }

        if (targetVarticalAngleOffset == null) {
            return Double.NaN;
        }

        Angle targetVerticalAngle = targetVarticalAngleOffset.add(LIMELIGHT_MOUNTING_PITCH);
        double targetHeight = APRIL_TAG_HEIGHTS[tagId];
        double heightOffset = targetHeight - LIMELIGHT_MOUNTING_POS.y;
        double distance = heightOffset / Math.tan(targetVerticalAngle.radians());

        return distance;
    }

    /**
     * Gets the Limelight network table.
     */
    private static NetworkTable table() {
        return NetworkTableInstance.getDefault().getTable("limelight");
    }

    /**
     * Whether the limelight has any valid targets (0 or 1)
     */
    private static double tv() {
        return table().getEntry("tv").getDouble(0.0);
    }

    /**
     * Horizontal Offset From Crosshair To Target 
     * (LL1: -27 degrees to 27 degrees / LL2: -29.8 to 29.8 degrees)
     */
    private static double tx() {
        return table().getEntry("tx").getDouble(0.0);
    }

    /**
     * Vertical Offset From Crosshair To Target 
     * (LL1: -20.5 degrees to 20.5 degrees / LL2: -24.85 to 24.85 degrees)
     */
    private static double ty() {
        return table().getEntry("ty").getDouble(0.0);
    }

    /**
     * Target Area (0% of image to 100% of image)
     */
    private static double ta() {
        return table().getEntry("ta").getDouble(0.0);
    }

    /**
     * Horizontal sidelength of the rough bounding box (0 - 320 pixels)
     */
    private static double thor() {
        return table().getEntry("thor").getDouble(0.0);
    }

    /**
     * Vertical sidelength of the rough bounding box (0 - 320 pixels)
     */
    private static double tvert() {
        return table().getEntry("tvert").getDouble(0);
    }

    /**
     * True active pipeline index of the camera (0 .. 9)
     */
    private static double getpipe() {
        return table().getEntry("getpipe").getDouble(0.0);
    }

    /**
     * Robot transform in field-space. Translation (X,Y,Z) 
     * Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
     */
    private static double[] botpose() {
        return table().getEntry("botpose").getDoubleArray(new double[6]);
    }

    /**
     * Robot transform in field-space (blue driverstation WPILIB origin).
     * Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
     */
    private static double[] botpose_wpiblue() {
        return table().getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    }

    /**
     * Robot transform in field-space (red driverstation WPILIB origin). 
     * Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
     */
    private static double[] botpose_wpired() {
        return table().getEntry("botpose_wpired").getDoubleArray(new double[6]);
    }

    /**
     * 3D transform of the camera in the coordinate system of the primary 
     * in-view AprilTag (array (6))
     */
    private static double[] camerapose_targetspace() {
        return table().getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
    }

    /**
     * 3D transform of the primary in-view AprilTag in the coordinate
     * system of the Camera (array (6))
     */
    private static double[] targetpose_cameraspace() {
        return table().getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
    }

    /**
     * 3D transform of the primary in-view AprilTag in the coordinate 
     * system of the Robot (array (6))
     */
    private static double[] targetpose_robotspace() {
        return table().getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
    }

    /**
     * 3D transform of the robot in the coordinate system of the primary 
     * in-view AprilTag (array (6))
     */
    private static double[] botpose_targetspace() {
        return table().getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    }

    /**
     * 3D transform of the camera in the coordinate system of the 
     * robot (array (6))
     */
    private static double[] camerapose_robotspace() {
        return table().getEntry("camerapose_robotspace").getDoubleArray(new double[6]);
    }

    /**
     * ID of the primary in-view AprilTag
     */
    private static double[] tid() {
        return table().getEntry("tid").getDoubleArray(new double[6]);
    }

    // Network tables and table entry docs taken from here:
    // https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
}
