package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Rack {
    private CANSparkMax motor;

    public Rack(int motorID) {
        motor = new CANSparkMax(motorID, MotorType.kBrushless);
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }
}
