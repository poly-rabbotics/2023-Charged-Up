package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;

public class Roller {
    private final PWMTalonSRX rollerMotor;

    public Roller(int rightRollerID) {
        rollerMotor = new PWMTalonSRX(rightRollerID);
    }

    public void setSpeed(double speed) {
        rollerMotor.set(speed);
    }
}
