package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.systems.ElevFourbar;
import frc.robot.systems.Intake;
import frc.robot.systems.ElevFourbar.Setpoint;
import frc.robot.systems.Intake.SolenoidState;

/** 
 * Class to control the fourbar mechanism 
 */
public class Fourbar {
    //Not currently utilized, may be implemented in the future
    private static final int FOURBAR_UPPER_LIMIT = 0;
    private static final int FOURBAR_LOWER_LIMIT = 140; 
    
    //PID max speed values
    private static final double FOURBAR_SPEED_UP = -0.5;
    private static final double FOURBAR_SPEED_DOWN = 0.5;

    //encoder offset
    private static final double ENCODER_OFFSET = 0.145 * 360;
    
    //constant variables
    private static final int MOTOR_ID = 62; //CORRECT ID
    private static final double MANUAL_DEADZONE = 0.3;

    //position constants, in degrees
    private static final int SUBSTATION_INTAKE_SETPOINT = 33 ;
    private static final int GROUND_INTAKE_UP_SETPOINT = 140;
    private static final int GROUND_INTAKE_DOWN_SETPOINT = 112;
    private static final int MID_SCORING_SETPOINT = 33;
    private static final int HIGH_SCORING_SETPOINT = 73;
    private static final int STOWED_SETPOINT = 0;

    //PID constants
    private static final double P = 5;
    private static final double I = 0.0;
    private static final double D = 1;
    private static final double F = 0.0;
    
    //Motor and controller
    private final CANSparkMax fourbarMotor;   
    private final SparkMaxPIDController pidController; 
    private SparkMaxAbsoluteEncoder absoluteEncoder;

    //variables
    private double targetSetpoint;
    private double encoderPosition;
    
    /**
     * Sets up fourbar motor and Xbox controller, configures PID
     */
    public Fourbar(){
        fourbarMotor = new CANSparkMax(MOTOR_ID, MotorType.kBrushless);

        absoluteEncoder = fourbarMotor.getAbsoluteEncoder(Type.kDutyCycle);
        pidController = fourbarMotor.getPIDController();
        
        pidController.setP(P);
        pidController.setI(I);
        pidController.setD(D);
        pidController.setFF(F);
        pidController.setOutputRange(FOURBAR_SPEED_UP, FOURBAR_SPEED_DOWN);

        pidController.setFeedbackDevice(absoluteEncoder);
        pidController.setPositionPIDWrappingEnabled(true);
        pidController.setPositionPIDWrappingMaxInput(360);
        pidController.setPositionPIDWrappingMinInput(0);

        fourbarMotor.setInverted(true);
        fourbarMotor.setIdleMode(IdleMode.kBrake);
    }

    public void autonomousRun(Setpoint setpoint) {
        encoderPosition = (absoluteEncoder.getPosition()*360) - ENCODER_OFFSET;

        updateTargetSetpoint(setpoint);

        pidControl(setpoint);
    }
    
    /**
     * Allows for cycling between setpoints using PID
     * @param setpoint The setpoint to move to, as defined in the Setpoint enum
     */
    public void pidControl(Setpoint setpoint){
        encoderPosition = (absoluteEncoder.getPosition()*360) - ENCODER_OFFSET;
        
        updateTargetSetpoint(setpoint);

        pidController.setReference((targetSetpoint + ENCODER_OFFSET) / 360.0, CANSparkMax.ControlType.kPosition);
    }

    /**
     * Allows for translating to setpoints using PID
     * @param coords The coordinates to move to, on an x and y plane
     */
    public void pidControl(double[] coords) {
        encoderPosition = (absoluteEncoder.getPosition()*360) - ENCODER_OFFSET;

        double[] pos = ElevFourbar.coordsToPos(coords[0], coords[1]);
        targetSetpoint = pos[1];

        /* if(Math.abs(targetSetpoint - 68) < 0.5) {
            targetSetpoint = 112; 
        } */

        pidController.setReference((targetSetpoint + ENCODER_OFFSET) / 360.0, CANSparkMax.ControlType.kPosition);
    }
    
    /**
     * Allows for manual control of motor output using the right joystick
     */
    public void manualControl(double speed){
        encoderPosition = (absoluteEncoder.getPosition()*360) - ENCODER_OFFSET;
        double outputSpeed;
        /* 
        //Restrict movement of fourarm to between upper and lower limit
        if(encoderPosition < FOURBAR_LOWER_LIMIT && speed < 0) {
            speed = 0;
        } else if(encoderPosition > FOURBAR_UPPER_LIMIT && speed > 0) {
            speed = 0;
        } */

        //Deadzone
        if(Math.abs(speed) < MANUAL_DEADZONE) {
            speed = 0;
        }

        //calculate gravity counteractment
        if(speed == 0) {
            double gravityBias = 0.07*Math.sin(encoderPosition*3.14159/180.0);
            outputSpeed = speed * 0.4-gravityBias;
        } else {
            outputSpeed = speed;
        }

        fourbarMotor.set(speed * 0.5);
        SmartDashboard.putNumber("Fourbar Power Ouput", fourbarMotor.getOutputCurrent());
    }

    private void updateTargetSetpoint(Setpoint setpoint) {
        switch (setpoint) {
            case SUBSTATION_INTAKE:
                targetSetpoint = SUBSTATION_INTAKE_SETPOINT;
                break;
            case GROUND_INTAKE:
                if(Intake.getPivotState() == SolenoidState.UP) {
                    targetSetpoint = GROUND_INTAKE_UP_SETPOINT;
                } else{
                    targetSetpoint = GROUND_INTAKE_DOWN_SETPOINT;
                }
                break;
            case MID_SCORING:
                targetSetpoint = MID_SCORING_SETPOINT;
                break;
            case HIGH_SCORING:
                targetSetpoint = HIGH_SCORING_SETPOINT;
                break;
            case STOWED:
                targetSetpoint = STOWED_SETPOINT;
                break;
        }
    }

    /**
     * 
     * @return Encoder position in degrees
     */
    public double getPosition() {
        return encoderPosition;
    }

    /**
     * 
     * @return Target encoder position in degrees
     */
    public double getTargetPosition() {
        return targetSetpoint;
    }
}