// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;

/** Add your docs here. */
public class Fourbar {
    private static Joystick joystick = new Joystick(0); //temporary controller, will be replaced with the universal controllers

    private static final int FOURBAR_UPPER_LIMIT = 90; //temp values, find the real ones in the CAD
    private static final int FOURBAR_LOWER_LIMIT = -40;
    //find these through testing
    private static final double PID_P = 0.0000001;
    private static final double PID_I = 0.0;
    private static final double PID_D = 0.0;
    //find this number through testing, this is the degrees that the entire 4bar moves in one rotation of the motor
    private static final double DEGREES_PER_MOTOR360 = 10; 
    //find this number through testing, this is the number of encoder counts in one rotation of the motor
    private static final double ENCODERCOUNTS_PER_360 = 5000; 

    //arbitrarily set, input the real values once we know them
    private static final int motorID = 7;
    private static final int encoderChannel = 9;
    private FourbarMode mode = FourbarMode.Manual;

    private enum FourbarMode {
        Manual, Position
    }


    private CANSparkMax fourbarMotor;
    private static DutyCycleEncoder absoluteEncoder;
    private static PIDController controller;
    private static double FOURBAR_DEGREES_PER_ENCODERCOUNTS = DEGREES_PER_MOTOR360 * ENCODERCOUNTS_PER_360;

    private static Fourbar instance = new Fourbar(motorID, encoderChannel);

    public Fourbar(int motorID, int encoderChannel){
        fourbarMotor = new CANSparkMax(motorID, MotorType.kBrushless);
        absoluteEncoder = new DutyCycleEncoder(encoderChannel);
        controller = new PIDController(PID_P, PID_I, PID_D);
        absoluteEncoder.setDistancePerRotation(360);
    }

    /**Commands the fourbar position in degrees */
    public void setPosition(double degrees){
        double setpoint = degrees / FOURBAR_DEGREES_PER_ENCODERCOUNTS;
        instance.fourbarMotor.set(controller.calculate(absoluteEncoder.getDistance(), setpoint));
    }

    public void manualControl(double speed){
        instance.fourbarMotor.set(speed);
    }
    /* this is a "subsubsystem" and should not have a run method, this is just for testing purposes and a template
    public static void run(){
        toggleMode();
        if (instance.mode == FourbarMode.Position)
        if(joystick.getRawButton(6)){
            instance.setPosition(FOURBAR_UPPER_LIMIT);
        } else if(joystick.getRawButton(7)){
            instance.setPosition(FOURBAR_LOWER_LIMIT);
        } 
        
        else {
            instance.manualControl(joystick.getY());
        }
    } 
    */

    public static void toggleMode(){
        if (joystick.getRawButtonPressed(3)) { //button is a placeholder, will be replaced with the universal controllers
            if (instance.mode == FourbarMode.Position){
                instance.mode = FourbarMode.Manual;
            } else {
                instance.mode = FourbarMode.Position;
            }
        }
        
    }


}