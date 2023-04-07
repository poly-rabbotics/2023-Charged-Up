// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.systems.AutoBalance;
import frc.robot.systems.Controls;
import frc.robot.systems.ElevFourbar;
import frc.robot.systems.Intake;
import frc.robot.systems.Pigeon;
import frc.robot.systems.SmartPrinter;
import frc.robot.systems.SwerveDrive;
import frc.robot.systems.AutoBalance.Stage;
import frc.robot.systems.ElevFourbar.GamePiece;
import frc.robot.systems.LEDLights;

import frc.robot.subsystems.SwerveMode;

import frc.robot.patterns.*;


/**
* The VM is configured to automatically run this class, and to call the functions corresponding to
* each mode, as described in the TimedRobot documentation. If you change the name of this class or
* the package after creating this project, you must also update the build.gradle file in the
* project.
*/
public class Robot extends TimedRobot {
    private static final XboxController controllerOne = (XboxController)Controls.getControllerByPort(0);
    private static final XboxController controllerTwo = (XboxController)Controls.getControllerByPort(1);
    private static final Joystick controlPanel = (Joystick)Controls.getControllerByPort(2);

    private static final AnalogInput pressureSensor = new AnalogInput(0);
    private static final DigitalInput brakeSwitch = new DigitalInput(1);

    /**
     * Custom turning curve for Rohan.
     */
    private static double turnCurveRohan(double x) {
        if (Math.abs(x) < 0.1) {
            return 0.0;
        }

        return Math.pow(x, 5);
    }

    /**
    * This function is run when the robot is first started up and should be used for any
    * initialization code.
    */
    @Override
    public void robotInit() {
        SwerveDrive.setTurnCurve(Robot::turnCurveRohan);
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
        
        SmartDashboard.putNumber("FB Position", ElevFourbar.fourbar.getPosition());
        SmartDashboard.putNumber("Comp Pressure", Math.floor(pressureValue));
        SmartDashboard.putBoolean("Fully Pressurized", pressureValue > 60);
        SmartDashboard.putNumber("Auto Mode", AutoModes.getAutoMode());
        
        SmartPrinter.print();
        LEDLights.run();
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
        Pigeon.setFeildZero();
        SwerveDrive.zeroPositions();
        LEDLights.setPatternIfNotEqual(new Breathe(new Color(0.8, 0.3, 0.0), 0.5));
        ElevFourbar.autonomousInit();

        // Reset Auto Balance to the idling stage in case autonomous has been 
        // run more than once since code start.
        AutoBalance.setStage(Stage.IDLING);
        AutoModes.init();
        Intake.init();
    }
    
    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        AutoModes.run();
    }
    
    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        SwerveDrive.setMode(SwerveMode.Headless);
        ElevFourbar.init();
        Intake.init();
    }
    
    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {        
        SmartDashboard.putNumber("controller Y", controllerOne.getLeftY());
        SmartDashboard.putNumber("controller X", controllerOne.getLeftX());

        double x = controllerOne.getLeftX();
        double y = controllerOne.getLeftY();

        if (controllerOne.getLeftTriggerAxis() > 0.25) {
            x = Math.abs(x) > Math.abs(y) ? x : 0.0;
            y = x == 0.0 ? y : 0.0;
        }

        SwerveDrive.run(x, y, controllerOne.getRightX(), controllerOne.getPOV());
        SwerveDrive.rockMode(controllerOne.getRightTriggerAxis() > 0.25);
        
        // Left stick changes between headless and relative control modes.
        if (controllerOne.getLeftStickButtonReleased()) {
            if (SwerveDrive.getMode() == SwerveMode.Headless) {
                SwerveDrive.setMode(SwerveMode.Relative);
            } else {
                SwerveDrive.setMode(SwerveMode.Headless);
            }
        }
        
        Intake.run(
            controlPanel.getRawButtonPressed(8), //controller one dpad to control pivot
            controlPanel.getRawButton(9),
            controlPanel.getRawButton(7),
            controlPanel.getRawButton(6),
            controlPanel.getRawButtonReleased(6)
        );
        
        ElevFourbar.run(
            controllerTwo.getRightY(),
            Math.abs(controlPanel.getRawAxis(0) / 2) > Math.abs(controllerTwo.getLeftY()) ? controlPanel.getRawAxis(0) / 2 : -controllerTwo.getLeftY(),
            controllerTwo.getPOV(),
            controlPanel.getRawButtonPressed(5), //toggle game piece
            controlPanel.getRawButton(1), //ground
            controlPanel.getRawButton(3), //mid
            controlPanel.getRawButton(4), //high
            controlPanel.getRawButton(2),  //stowed
            controllerTwo.getStartButtonPressed() //zero elevator encoder
        );

        //Change color base on which gamepiece we have selected
        if(ElevFourbar.gamePieceSelected == GamePiece.CONE) {
            LEDLights.setPatternIfNotEqual(new Breathe(yellow, 0.5));
        } else {
            LEDLights.setPatternIfNotEqual(new Breathe(purple, 0.5));
        }
    }
    
    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        LEDLights.setPatternIfNotEqual(new Rainbow(LEDLights.LED_LENGTH, 50));
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
