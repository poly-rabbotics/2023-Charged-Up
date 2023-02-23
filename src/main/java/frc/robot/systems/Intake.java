package frc.robot.systems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.*;

public class Intake {
    private static final int RACK_MOTOR_ID = 11;
    private static final int LEFT_ROLLER_MOTOR_ID = 0;
    private static final int RIGHT_ROLLER_MOTOR_ID = 0;
    private static final double ROLLER_DEADZONE = 0.3;

    private double rackMotorSpeed;
    private boolean clawOpen;
    private boolean pivotDown;

    
    private DoubleSolenoid clawSolenoid;
    private Compressor comp;

    private static Intake instance = new Intake();
    private static Rack rack;
    private static Pivot pivot;
    private static Roller roller;

    public Intake() {
        rack = new Rack(RACK_MOTOR_ID);

        //UNCOMMENT LATER
        //roller = new Roller(RIGHT_ROLLER_ID, LEFT_ROLLER_ID);
        //clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 0);
        pivot = new Pivot(PneumaticsModuleType.CTREPCM, 5, 4);
        comp = new Compressor(1, PneumaticsModuleType.CTREPCM);
        comp.enableDigital();
    }

    public static void init() {
        instance.rackMotorSpeed = 0;
        instance.clawOpen = false;
        instance.pivotDown = false;
    }

    /**
     * Runs the rack motor, operated with DPAD left/right
     */
    public static void runRack(int dPadDirection) {
        if(dPadDirection == 90) {
            instance.rackMotorSpeed = 0.8;
        } else if(dPadDirection == 270) {
            instance.rackMotorSpeed = -0.8;
        } else {
            instance.rackMotorSpeed = 0;
        }

        rack.setSpeed(instance.rackMotorSpeed);

        SmartDashboard.putNumber("Rack Motor Speed", instance.rackMotorSpeed);
        SmartDashboard.putNumber("POV", dPadDirection);
    }

    /**
     * Runs the rollers, operated with right joystick Y axis
     */
    private static void runRoller(double rollerSpeed) {
        if(Math.abs(rollerSpeed) > ROLLER_DEADZONE) { // scales down the speed of the motor
            rollerSpeed *= 0.8;
        } else {
            rollerSpeed = 0;
        }

        //roller.setRoller(rollerSpeed)
    }

    private static void runClaw(boolean xButtonPressed) {
        if(xButtonPressed) {
            if(!instance.clawOpen) {
                //instance.clawSolenoid.set(Value.kReverse);
                instance.clawOpen = true;
            } else {
                //instance.clawSolenoid.set(Value.kForward);
                instance.clawOpen = false;
            }
        } 
    }  
    
    public static void runPivot(boolean leftBumperPressed) {
        if(leftBumperPressed) {
            if(!instance.pivotDown) {
                pivot.setPivot(Value.kReverse);
                instance.pivotDown = true;
            } else {
                pivot.setPivot(Value.kForward);
                instance.pivotDown = false;
            }
        }
    }
    
}
