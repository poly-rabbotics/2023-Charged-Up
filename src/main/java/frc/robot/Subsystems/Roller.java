package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Roller {
    TalonSRX rollerMotor;
    TalonSRX leftRollerMotor;

    public Roller(int rightRollerID) {
        rollerMotor = new TalonSRX(rightRollerID);
    }

    public void setSpeed(double speed) {
        rollerMotor.set(ControlMode.PercentOutput, speed);
    }
}
