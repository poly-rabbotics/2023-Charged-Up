// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.ModuleLayer.Controller;
import java.util.ResourceBundle.Control;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

/** Add your docs here. */
public class DriveUnit {
    private static final double PID_OUTPUT_MAX = 1;
    private static final double PID_OUTPUT_MIN = -1;
    private CANSparkMax leader;
    private CANSparkMax follower;
    private RelativeEncoder encoder;
    private SparkMaxPIDController controller;
    
    
    /**
     * Creates new drive unit
     * @param leaderCANID CAN ID for the leader NEO
     * @param followerCANID CAN ID for the follower NEO
     */
    public DriveUnit(int leaderCANID, int followerCANID) {
        leader = new CANSparkMax(leaderCANID, MotorType.kBrushless);
        follower = new CANSparkMax(followerCANID, MotorType.kBrushless);
        follower.follow(leader);

        encoder = leader.getEncoder();
        controller = leader.getPIDController();
    }
    
    /**
     * sets the speed for the leader CANID
     * @param speed
     */
    public void set(double speed) {
        leader.set(speed);
    }
    public void setController(SparkMaxPIDController newController) {
        controller = newController;
    }

    public SparkMaxPIDController getController() {
        return controller;
    }

    public void settupPID(double p, double i, double d) {
        controller.setP(p);
        controller.setI(i);
        controller.setD(d);
        
        controller.setOutputRange(PID_OUTPUT_MIN, PID_OUTPUT_MAX);
    }
}
