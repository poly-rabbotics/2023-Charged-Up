package frc.robot.systems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    /**
     * Whether the limelight has any valid targets (0 or 1)
     */
    private static final NetworkTableEntry tv = table.getEntry("tv");

    /**
     * Horizontal Offset From Crosshair To Target 
     * (LL1: -27 degrees to 27 degrees / LL2: -29.8 to 29.8 degrees)
     */
    private static final NetworkTableEntry tx = table.getEntry("tx");

    /**
     * Vertical Offset From Crosshair To Target 
     * (LL1: -20.5 degrees to 20.5 degrees / LL2: -24.85 to 24.85 degrees)
     */
    private static final NetworkTableEntry ty = table.getEntry("ty");

    /**
     * Target Area (0% of image to 100% of image)
     */
    private static final NetworkTableEntry ta = table.getEntry("ta");

    /**
     * Horizontal sidelength of the rough bounding box (0 - 320 pixels)
     */
    private static final NetworkTableEntry thor = table.getEntry("thor");

    /**
     * Vertical sidelength of the rough bounding box (0 - 320 pixels)
     */
    private static final NetworkTableEntry tvert = table.getEntry("tvert");

    /**
     * True active pipeline index of the camera (0 .. 9)
     */
    private static final NetworkTableEntry getpipe = table.getEntry("getpipe");

    /**
     * Robot transform in field-space. Translation (X,Y,Z) 
     * Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
     */
    private static final NetworkTableEntry botpose = table.getEntry("botpose");

    /**
     * Robot transform in field-space (blue driverstation WPILIB origin).
     * Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
     */
    private static final NetworkTableEntry botpose_wpiblue = table.getEntry("botpose_wpiblue");

    /**
     * Robot transform in field-space (red driverstation WPILIB origin). 
     * Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
     */
    private static final NetworkTableEntry botpose_wpired = table.getEntry("botpose_wpired");

    /**
     * 3D transform of the camera in the coordinate system of the primary 
     * in-view AprilTag (array (6))
     */
    private static final NetworkTableEntry camerapose_targetspace = table.getEntry("camerapose_targetspace");

    /**
     * 3D transform of the primary in-view AprilTag in the coordinate
     * system of the Camera (array (6))
     */
    private static final NetworkTableEntry targetpose_cameraspace = table.getEntry("targetpose_cameraspace");

    /**
     * 3D transform of the primary in-view AprilTag in the coordinate 
     * system of the Robot (array (6))
     */
    private static final NetworkTableEntry targetpose_robotspace = table.getEntry("targetpose_robotspace");

    /**
     * 3D transform of the robot in the coordinate system of the primary 
     * in-view AprilTag (array (6))
     */
    private static final NetworkTableEntry botpose_targetspace = table.getEntry("botpose_targetspace");

    /**
     * 3D transform of the camera in the coordinate system of the 
     * robot (array (6))
     */
    private static final NetworkTableEntry camerapose_robotspace = table.getEntry("camerapose_robotspace");

    /**
     * ID of the primary in-view AprilTag
     */
    private static final NetworkTableEntry tid = table.getEntry("tid");

    // Network tables and table entry docs taken from here:
    // https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
    

}
