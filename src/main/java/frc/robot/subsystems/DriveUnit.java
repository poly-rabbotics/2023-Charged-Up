// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** Add your docs here. */
public class DriveUnit {
    private CANSparkMax leader;
    private CANSparkMax follower;
    
    /**
     * Creates new drive unit
     * @param leaderCANID CAN ID for the leader NEO
     * @param followerCANID CAN ID for the follower NEO
     */
    public DriveUnit(int leaderCANID, int followerCANID) {
        leader = new CANSparkMax(leaderCANID, MotorType.kBrushless);
        follower = new CANSparkMax(followerCANID, MotorType.kBrushless);
        follower.follow(leader);
    }
    /**
     * sets the speed for the leader CANID
     * @param speed
     */
    public void set(double speed) {
        leader.set(speed);
    }
}
