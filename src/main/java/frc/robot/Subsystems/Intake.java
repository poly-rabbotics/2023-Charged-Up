package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.fasterxml.jackson.databind.jsontype.PolymorphicTypeValidator.Validity;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import pabeles.concurrency.IntRangeConsumer;;

public class Intake {
    private final int RACK_MOTOR_ID = 0;
    private final int LEFT_ROLLER_MOTOR_ID = 0;
    private final int RIGHT_ROLLER_MOTOR_ID = 0;
    private final double ROLLER_DEADZONE = 0.3;

    private double rackMotorSpeed;
    private double rollerSpeed;
    private boolean clawOpen;
    private boolean pivotDown;

    private TalonSRX leftRollerMotor;
    private TalonSRX rightRollerMotor;
    private CANSparkMax rackMotor;
    private XboxController controller;
    private DoubleSolenoid clawSolenoid;
    private DoubleSolenoid pivotSolenoid;

    private static Intake instance = new Intake();

    public Intake() {
        rackMotor = new CANSparkMax(0, MotorType.kBrushless);

        leftRollerMotor = new TalonSRX(LEFT_ROLLER_MOTOR_ID);
        rightRollerMotor = new TalonSRX(RIGHT_ROLLER_MOTOR_ID);

        controller = new XboxController(0);
    }

    /**
     * Method to be run from teleopPeriodic
     */
    private void run() {

    }

    public static void runRack() {
        if(instance.controller.getPOV() == 90) {
            instance.rackMotorSpeed = 0.2;
        } else if(instance.controller.getPOV() == 270) {
            instance.rackMotorSpeed = -0.2;
        } else {
            instance.rackMotorSpeed = 0;
        }

        instance.rackMotor.set(instance.rackMotorSpeed);
    }

    private void runRoller() {
        if(Math.abs(instance.controller.getRightY()) > ROLLER_DEADZONE) {
            rollerSpeed = instance.controller.getRightY() * 0.8;
        } else {
            rollerSpeed = 0;
        }

        rightRollerMotor.set(ControlMode.PercentOutput, rollerSpeed);
    }

    private void runClaw() {
        if(instance.controller.getXButton()) {

            if(!instance.clawOpen) {
                instance.clawSolenoid.set(Value.kReverse);
                instance.clawOpen = true;
            } else {
                instance.clawSolenoid.set(Value.kForward);
                instance.clawOpen = false;
            }
        } 
    }  
    private void runPivot() {
        if(instance.controller.getLeftBumper()) {
            if(!instance.pivotDown) {
                instance.pivotSolenoid.set(Value.kReverse);
                instance.pivotDown = true;
            } else {
                instance.pivotSolenoid.set(Value.kForward);
                instance.pivotDown = false;
            }
        }
    }
    
}
