package frc.robot.Subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;

public class Test {
    PWMTalonSRX motor;
    Joystick stick;   
    private static Test instance = new Test();
    private Test() {
        motor = new PWMTalonSRX(0);
        stick = new Joystick(0);

    }

    public static void run() {
        double speed = instance.stick.getRawAxis(1);
        instance.motor.set(speed);
    }

}
