package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.systems.ElevFourbar.Setpoint;
import frc.robot.systems.ElevFourbar;

/**
 * Controls the elevator
 */
public class Elevator {
    //Ticks per inch constant, gear reduction is 12:1
    private static final double TICKS_PER_INCH = -6144.0; //FINAL VALUE DO NOT CHANGE
    
    //constant variables
    private static final double MANUAL_DEADZONE = 0.3;
    private static final int ELEVATOR_MOTOR_ID = 62; //CORRECT ID
    
    //position constants, in inches
    private static final double SUBSTATION_INTAKE_SETPOINT = 9;
    private static final double GROUND_INTAKE_SETPOINT = 30;
    private static final double MID_SCORING_SETPOINT = 11;
    private static final double HIGH_SCORING_SETPOINT = 30;
    private static final double STOWED_SETPOINT = 0;
    
    //PID constants
    private static final double P = 1.0;
    private static final double I = 0.00001;
    private static final double D = 0.01;
    
    //Motor and controller
    private final TalonFX elevatorMotor;

    //variables
    private double encoderPosition;
    private double targetSetpoint;

    private DigitalInput limit;
    
    /**
     * Sets up elevator motor and Xbox controller, configures PID
     */
    public Elevator() {
        elevatorMotor = new TalonFX(ELEVATOR_MOTOR_ID);

        elevatorMotor.configFactoryDefault();
        elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
        elevatorMotor.config_kP(0, P);
        elevatorMotor.config_kI(0, I);
        elevatorMotor.config_kD(0, D);
        elevatorMotor.selectProfileSlot(0, 0);

        elevatorMotor.configPeakOutputForward(0.85);
        elevatorMotor.configPeakOutputReverse(-0.85);
        
        //Configures motor to brake when not being used
        elevatorMotor.setNeutralMode(NeutralMode.Brake);
        limit = new DigitalInput(0);
    }

    /**
     * The method to be run in teleopInit to reset variables
     */
    public void init() {
    }

    public void autonomousInit() {
        targetSetpoint = STOWED_SETPOINT;
    }

    /**
     * Manual control of the elevator using the left joystick
     * @param speed The speed of the elevator from 0 to 1
     * @param dPadDirection The value of the dpad
     */
    public void manualControl(double speed, int dPadDirection) {
        encoderPosition = elevatorMotor.getSensorCollection().getIntegratedSensorPosition();

        if (Math.abs(speed) < MANUAL_DEADZONE) {
            speed = 0;
        } 
        
        //low sensitivity control using DPAD
        if (dPadDirection == 180) {
            speed = 0.2;
        } else if (dPadDirection == 0) {
            speed = -0.2;
        }

        if (!limit.get()) {
            if (speed > 0.0) {
                speed = 0.0;
            }
        }

        elevatorMotor.set(ControlMode.PercentOutput, speed * 0.6);
    }

    /**
     * PID Control of the elevator
     * @param setpoint The setpoint to move the elevator to
     */
    public void pidControl(Setpoint setpoint) {
        encoderPosition = elevatorMotor.getSensorCollection().getIntegratedSensorPosition();

        updateTargetSetpoint(setpoint);

        //set elevator PID position to target setpoint
        elevatorMotor.set(ControlMode.Position, targetSetpoint * TICKS_PER_INCH);
    }

    /**
     * PID Control of the elevator using coordinates
     * @param coords The coordinates to move to, on an x and y plane
     */
    public void pidControl(double[] coords) {
        encoderPosition = elevatorMotor.getSensorCollection().getIntegratedSensorPosition();

        double[] pos = ElevFourbar.coordsToPos(coords[0], coords[1]);
        targetSetpoint = pos[0];

        if(coords[0] == 35.2) {
            targetSetpoint = 31;
        } else if (coords == ElevFourbar.STOWED_COORDS_CONE) {
            targetSetpoint = 0.0;
        }

        /* if(Math.abs(targetSetpoint - 2.9) < 0.5) {
            targetSetpoint = GROUND_INTAKE_SETPOINT;
        } */
        
        //set elevator PID position to target setpoint
        elevatorMotor.set(ControlMode.Position, targetSetpoint * TICKS_PER_INCH);
    }

    /**
     * 
     * @return Encoder position in inches
     */
    public double getPosition() {
        return encoderPosition / TICKS_PER_INCH;
    }

    /**
     * 
     * @return Target encoder position in inches
     */
    public double getTargetPosition() {
        return targetSetpoint;
    }

    private void updateTargetSetpoint(Setpoint setpoint) {
        switch (setpoint) {
            case SUBSTATION_INTAKE:
                targetSetpoint = SUBSTATION_INTAKE_SETPOINT;
                break;
            case GROUND_INTAKE:
                if(ElevFourbar.fourbar.getPosition() > 5)
                    targetSetpoint = GROUND_INTAKE_SETPOINT;
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

    public void zeroEncoder() {
        elevatorMotor.getSensorCollection().setIntegratedSensorPosition(elevatorMotor.getSensorCollection().getIntegratedSensorPosition() + 100, 30);
    }
}
