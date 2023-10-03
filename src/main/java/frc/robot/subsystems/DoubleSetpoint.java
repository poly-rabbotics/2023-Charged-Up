package frc.robot.subsystems;

/**
 * Enables the ability to store both cube 
 */
public class DoubleSetpoint extends Setpoint {
    public Setpoint cube;
    public Setpoint cone;

    public DoubleSetpoint(Setpoint cube, Setpoint cone) {
        super(0, 0);

        this.cube = cube;
        this.cone = cone;
    }
}