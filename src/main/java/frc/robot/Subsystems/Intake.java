package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.fasterxml.jackson.databind.jsontype.PolymorphicTypeValidator.Validity;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import pabeles.concurrency.IntRangeConsumer;;

public class Intake {
    private static final int RACK_MOTOR_ID = 11;
    private static final int LEFT_ROLLER_MOTOR_ID = 0;
    private static final int RIGHT_ROLLER_MOTOR_ID = 0;
    private static final double ROLLER_DEADZONE = 0.3;

    private double rackMotorSpeed;
    private boolean clawOpen;
    private boolean pivotDown;

    private TalonSRX leftRollerMotor;
    private TalonSRX rightRollerMotor;
    private CANSparkMax rackMotor;
    private DoubleSolenoid clawSolenoid;
    private DoubleSolenoid pivotSolenoid;
    private Compressor comp;

    private static Intake instance = new Intake();

    public Intake() {
        rackMotor = new CANSparkMax(RACK_MOTOR_ID, MotorType.kBrushless);

        //UNCOMMENT LATER
        //leftRollerMotor = new TalonSRX(LEFT_ROLLER_MOTOR_ID);
        //rightRollerMotor = new TalonSRX(RIGHT_ROLLER_MOTOR_ID);
        //clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 0);
        pivotSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 5, 4);
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
            instance.rackMotorSpeed = 0.2;
        } else if(dPadDirection == 270) {
            instance.rackMotorSpeed = -0.2;
        } else {
            instance.rackMotorSpeed = 0;
        }

        instance.rackMotor.set(instance.rackMotorSpeed);

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

        //instance.rightRollerMotor.set(ControlMode.PercentOutput, rollerSpeed);

        //makes the left roller follow the right roller
        //instance.leftRollerMotor.follow(instance.rightRollerMotor);
        //instance.leftRollerMotor.setInverted(true);
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
                instance.pivotSolenoid.set(Value.kReverse);
                instance.pivotDown = true;
            } else {
                instance.pivotSolenoid.set(Value.kForward);
                instance.pivotDown = false;
            }
        }
    }
    
}
