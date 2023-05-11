package frc.robot.subsystems;

/**
 * Class to simplify the use of setpoints so I don't
 * have to deal with even more methods in ElevFourbar
 */
public class Setpoint {

    public static final double HYPOTENUSE = 37.5;
    public static final double ELEVATOR_MAX = 32;

    public static final double BUMPER_X = 14;
    public static final double BUMPER_Y = 6.5;
    
    //Instance variables
    private double[] coords; //Formatted as [ x, y ]
    private double elevPos;
    private double fourbarPos;
    private boolean isNegative;
    
    /**
     * isNeg forces the fourbar to be angled down (if possible)
     * just to help with the occasional ambiguity of positions
     */
    public Setpoint(double x, double y, boolean isNegative) {
        double[] coords = { x, y };
        this.coords = coords;
        this.isNegative = isNegative;

        double[] pos = toPositions(coords, isNegative);
        this.elevPos = pos[0];
        this.fourbarPos = pos[1];
    }

    /**
     * Initialize a setpoint without override of position
     */
    public Setpoint(double x, double y) {
        this(x, y, false);
    }

    public Setpoint(boolean uselessBooleanToMakeThisADifferentConstructor, double e, double f) {
        this.elevPos = e;
        this.fourbarPos = f;
        this.coords = toCoords(e, f);
    }

    /**
     * Ew dont use this one at least make use of my hard work :(
     */
    /* public Setpoint(double elevPos, double fourbarPos) {
        this.elevPos = elevPos;
        this.fourbarPos = fourbarPos;

        this.coords = toCoords(elevPos, fourbarPos);
        this.isNegative = false;
    } */

    /**
     * @return ( x, y ) coordinates of the setpoint
     */
    public double[] getCoords() {
        return coords;
    }

    /**
     * @return elevator position of the setpoint in inches
     */
    public double getElevPos() {
        return elevPos;
    }

    /**
     * @return fourbar position of the setpoint in degrees
     */
    public double getFourbarPos() {
        return fourbarPos;
    }

    /**
     * @return true if the fourbar is forced to angle down
     */
    public boolean getIsNegative() {
        return isNegative;
    }

    /**
     * @param elevPos
     * @param fourbarPos
     * @return Coordinates resulting from the inputted positions
     */
    public static double[] toCoords(double elevPos, double fourbarPos) {

        double x;
        double y;

        x = Math.sin(Math.toRadians(fourbarPos)) * HYPOTENUSE;
        y = Math.cos(Math.toRadians(fourbarPos)) * HYPOTENUSE + elevPos;

        //I had to make a variable for the output because it didn't just let me return { x, y}
        double[] res = { x, y };
        return res;
    }

    /**
     * This thing works now!!
     * If it doesnt return the correct values, try modifying 
     * the bumper coordinates or the elevator max
     * @param coord x, y coordinates to convert from
     * @param isNegative true if the fourbar needs to be forced to angle down
     * @return Positions in the format of { elevator, fourbar }
     */
    public static double[] toPositions(double[] coord, boolean isNegative) {

        double x = coord[0];
        double y = coord[1];
        
        double e;
        double f;

        //JESUS CHRIST WHY WAS THIS PART SO HARD
        double adj = Math.sqrt((Math.pow(HYPOTENUSE, 2) - Math.pow(x, 2)));
        e = y < adj || !approx(y, adj, 0.0001) || isNegative 
            ? y + adj 
            : y - adj;

        e = clamp(e, 0, ELEVATOR_MAX);

        //atan2 my beloved <3
        f = Math.toDegrees(Math.atan2(x, y - e));

        //Limit the angle so we dont break the bumper on accident
        double maxAngle = Math.toDegrees(Math.atan2(BUMPER_X, BUMPER_Y - e));
        f = clamp(f, 0, maxAngle);

        double[] res = { e, f };
        return res;
    }

    /**
     * Used to clamp the range of variables so I don't bend
     * any more steel hex shafts :)
     */
    private static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    /**
     * 
     * @param a
     * @param b
     * @param sensitivity
     * @return true if the difference between a and b is less than sensitivity
     */
    private static boolean approx(double a, double b, double sensitivity) {
        return Math.abs(a - b) < sensitivity;
    }
}
