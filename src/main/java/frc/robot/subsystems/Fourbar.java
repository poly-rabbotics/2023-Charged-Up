// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

/** Add your docs here. */
public class Fourbar {
    private static final int FOURBAR_UPPER_LIMIT = 10;
    private static final int FOURBAR_LOWER_LIMIT = 10;

    private static final double PID_P = 0.0000001;
    private static final double PID_I = 0.0;
    private static final double PID_D = 0.0;


    private CANSparkMax motor;
    private DutyCycleEncoder absoluteEncoder;
    private PIDController controller;

    public Fourbar(int motorID, int encoderChannel){
        motor = new CANSparkMax(motorID, MotorType.kBrushless);
        absoluteEncoder = new DutyCycleEncoder(encoderChannel);
        controller = new PIDController(PID_P, PID_I, PID_D);
    }

    public void setPosition(double x){
        motor.set(controller.calculate(absoluteEncoder.getDistance(), x));
    }

    //assuming that y is greater than 0 and less than 1
    public void fourbarUp(double y){
        motor.set(y);
    }

    //assumes that y is less than 0 and greater than -1
    public void fourbarDown(double y){
        motor.set(y);
    }


}