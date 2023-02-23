package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Rack {
    private TalonSRX motor;

    public Rack(int motorID) {
        motor = new TalonSRX(motorID);
    }

    public void setSpeed(double speed) {
        motor.set(ControlMode.PercentOutput, speed);
    }
}
