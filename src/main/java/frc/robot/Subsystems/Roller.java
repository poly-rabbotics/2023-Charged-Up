package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Roller {
    TalonSRX rightRollerMotor;
    TalonSRX leftRollerMotor;

    public Roller(int rightRollerID, int leftRollerID) {
        rightRollerMotor = new TalonSRX(rightRollerID);
        leftRollerMotor = new TalonSRX(leftRollerID);
    }

    public void setSpeed(double speed) {
        rightRollerMotor.set(ControlMode.PercentOutput, speed);

        leftRollerMotor.follow(rightRollerMotor);
        leftRollerMotor.setInverted(InvertType.InvertMotorOutput);
    }
}
