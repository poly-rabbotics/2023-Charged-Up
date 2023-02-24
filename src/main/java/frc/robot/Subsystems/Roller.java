package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;

public class Roller {
    PWMTalonSRX rollerMotor;
    PWMTalonSRX leftRollerMotor;

    public Roller(int rightRollerID) {
        rollerMotor = new PWMTalonSRX(rightRollerID);
    }

    public void setSpeed(double speed) {
        rollerMotor.set(speed);
    }
}
