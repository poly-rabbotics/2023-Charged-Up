package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;

public class Rack {
    private PWMTalonSRX motor;

    public Rack(int motorID) {
        motor = new PWMTalonSRX(motorID);
    }
    
    public void setSpeed(double speed) {
        motor.set(speed);
    }
}