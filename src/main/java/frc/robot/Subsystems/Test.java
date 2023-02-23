package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;

public class Test {
    TalonSRX motor;
    Joystick stick;   

    public Test() {
        motor = new TalonSRX(0);
        stick = new Joystick(0);

    }

    public void run() {
        double speed = stick.getRawAxis(1);
        motor.set(ControlMode.PercentOutput, speed);
    }

}
