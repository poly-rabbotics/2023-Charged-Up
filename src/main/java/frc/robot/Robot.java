// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// BIGGER CHANGES

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.systems.AutoBalance;
import frc.robot.systems.AutonomousRunner;
import frc.robot.systems.Controls;
import frc.robot.systems.ElevFourbar;
import frc.robot.systems.Intake;
import frc.robot.systems.Pigeon;
import frc.robot.systems.SmartPrinter;
import frc.robot.systems.SwerveDrive;
import frc.robot.systems.AutoBalance.Stage;
import frc.robot.systems.LEDLights;
import frc.robot.systems.Bat;
import frc.robot.subsystems.SwerveMode;

/**
* The VM is configured to automatically run this class, and to call the functions corresponding to
* each mode, as described in the TimedRobot documentation. If you change the name of this class or
* the package after creating this project, you must also update the build.gradle file in the
* project.
*/
public class Robot extends TimedRobot {
    public enum ControlMode {
        DISABLED,
        AUTONOMOUS,
        TELEOPERATED,
        SAFETY
    }

    private static final double DRIVE_ACCELERATION_LIMIT = 1.5;

    private static final XboxController controllerOne = (XboxController)Controls.getControllerByPort(0);
    private static final XboxController controllerTwo = (XboxController)Controls.getControllerByPort(1);
    private static final Joystick controlPanel = (Joystick)Controls.getControllerByPort(2);

    private static final AnalogInput pressureSensor = new AnalogInput(0);
    private static final DigitalInput brakeSwitch = new DigitalInput(1);

    private static Robot instance;

    private ControlMode controlMode = ControlMode.DISABLED;

    /**
     * Exists only to enable static methods to gain access to non static data,
     * OOP fans be damned I just made your class a singleton.
     */
    public Robot() {
        super();
        instance = this;
    }

    public static ControlMode getControlMode() {
        return instance.controlMode;
    }

    /**
    * This function is run when the robot is first started up and should be used for any
    * initialization code.
    */
    @Override
    public void robotInit() {
        controlMode = ControlMode.DISABLED;
        SwerveDrive.setTurnCurve(Controls::defaultCurve);
        SwerveDrive.setDirectionalCurve(Controls::defaultCurveTwoDimensional);
    }
    
    /**
    * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
    * that you want ran during disabled, autonomous, teleoperated and test.
    *
    * <p>This runs after the mode specific periodic functions, but before LiveWindow and
    * SmartDashboard integrated updating.
    */
    @Override
    public void robotPeriodic() {
        ElevFourbar.setFourbarBrake(brakeSwitch.get());
        double pressureValue = (pressureSensor.getValue() - 410) / 13.5;
        
        SmartDashboard.putNumber("Comp Pressure", Math.floor(pressureValue));
        SmartDashboard.putBoolean("Fully Pressurized", pressureValue > 60);
        
        SmartPrinter.print();
        LEDLights.run();
        Bat.getRange();
    }
    
    /**
    * This autonomous (along with the chooser code above) shows how to select between different
    * autonomous modes using the dashboard. The sendable chooser code works with the Java
    * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
    * uncomment the getString line to get the auto name from the text box below the Gyro
    *
    * <p>You can add additional auto modes by adding additional comparisons to the switch structure
    * below with additional strings. If using the SendableChooser make sure to add them to the
    * chooser code above as well.
    */
    @Override
    public void autonomousInit() {
        controlMode = ControlMode.AUTONOMOUS;

        int autoMode = 
            (controlPanel.getRawButton(12) ? 1 << 0 : 0) + 
            (controlPanel.getRawButton(11) ? 1 << 1 : 0) + 
            (controlPanel.getRawButton(10) ? 1 << 2 : 0);
        
        AutonomousRunner.init();
        AutonomousRunner.setAutoMode(autoMode);

        Pigeon.setFeildZero();
        SwerveDrive.zeroPositions();
        SwerveDrive.setAccelerationRate(Double.NaN);
        //ElevFourbar.autonomousInit();
        ElevFourbar.init();
        Intake.init();

        // Reset Auto Balance to the idling stage in case autonomous has been 
        // run more than once since code start.
        AutoBalance.setStage(Stage.IDLING);
    }
    
    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        AutonomousRunner.run();
    }
    
    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        controlMode = ControlMode.TELEOPERATED;
        SwerveDrive.setMode(SwerveMode.HEADLESS);
        SwerveDrive.setAccelerationRate(Double.NaN);
        ElevFourbar.init();
        Intake.init();
    }
    
    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {        
        // Left stick changes between headless and relative control modes.
        if (controllerOne.getLeftStickButtonReleased()) {
            SwerveDrive.setMode(
                SwerveDrive.getMode() == SwerveMode.HEADLESS 
                    ? SwerveMode.RELATIVE 
                    : SwerveMode.HEADLESS
            );
        }

        if (controllerOne.getRightStickButtonReleased()) {
            var accelerationRate = SwerveDrive.getAccelerationRate();
            SwerveDrive.setAccelerationRate(
                accelerationRate == accelerationRate
                    ? Double.NaN
                    : DRIVE_ACCELERATION_LIMIT
            );
        }
        
        SwerveDrive.conditionalTempDirectionalCurve(
            Controls.cardinalLock(Controls::defaultCurveTwoDimensional), 
            controllerOne.getXButton()
        ); // Lock to cardinal directions.
        SwerveDrive.conditionalTempDirectionalCurve(
            (x, y) -> Controls.defaultCurveTwoDimensional(x, y) / 3.0,
            controllerOne.getRightBumper()
        ); // Half translation speed.
        SwerveDrive.conditionalTempDirectionalCurve(
            Controls.cardinalLock((x, y) -> Controls.defaultCurveTwoDimensional(x, y) / 2.0),
            controllerOne.getRightBumper() && controllerOne.getXButton()
        ); // Half translation speed.
        SwerveDrive.conditionalTempMode(SwerveMode.ROCK, controllerOne.getBButton());
        SwerveDrive.run(
            controllerOne.getLeftX(),
            controllerOne.getLeftY(),
            //controllerOne.getRightTriggerAxis() - controllerOne.getLeftTriggerAxis(),
            controllerOne.getRightX()
        );

        double rumble = controllerOne.getLeftBumper() 
            ? SwerveDrive.getAverageMotorTemp() / 80.0
            : SwerveDrive.getAveragePercentRatedCurrent();
        controllerOne.setRumble(RumbleType.kBothRumble, rumble);
        
        Intake.run(
            controlPanel.getRawButtonPressed(8), //Pivot toggle
            controlPanel.getRawButton(9), //Claw hold
            controlPanel.getRawButton(6), //Intake hold
            controlPanel.getRawButton(7) //Outtake hold
        );

        ElevFourbar.run(
            controllerTwo.getRightY(), //Manual elevator
            Math.abs(controlPanel.getRawAxis(0) / 2) > Math.abs(controllerTwo.getLeftY()) ? controlPanel.getRawAxis(0) / 2 : -controllerTwo.getLeftY(), //Manual fourbar
            controllerTwo.getPOV(), //DPad direction
            controlPanel.getRawButtonPressed(5), //Toggle game piece
            controlPanel.getRawButton(1), //Ground intake
            controlPanel.getRawButton(3), //Mid scoring
            controlPanel.getRawButton(4), //High scoring
            controllerTwo.getStartButtonPressed() || controlPanel.getRawButton(2) //Zero elevator encoder
        );
    }
    
    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        controlMode = ControlMode.DISABLED;
    }
    
    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
        ElevFourbar.toggleGamePiece(controlPanel.getRawButtonReleased(5)); 
    }
    
    /** This  function is called once when test mode is enabled. */
    @Override
    public void testInit() {}
    
    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}
  
    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}
    
    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
