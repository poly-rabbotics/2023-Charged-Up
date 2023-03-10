// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import java.util.concurrent.*;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.SwerveMode;
import frc.robot.systems.ElevFourbar;
import frc.robot.systems.Intake;
import frc.robot.systems.Pigeon;
import frc.robot.systems.SwerveDrive;
import frc.robot.systems.LEDLights;
import frc.robot.patterns.*;

/**
* The VM is configured to automatically run this class, and to call the functions corresponding to
* each mode, as described in the TimedRobot documentation. If you change the name of this class or
* the package after creating this project, you must also update the build.gradle file in the
* project.
*/
public class Robot extends TimedRobot {
    public static XboxController controllerOne = new XboxController(0);
    public static XboxController controllerTwo = new XboxController(1);
    
    /**
    * This function is run when the robot is first started up and should be used for any
    * initialization code.
    */
    @Override
    public void robotInit() {}
    
    /**
    * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
    * that you want ran during disabled, autonomous, teleoperated and test.
    *
    * <p>This runs after the mode specific periodic functions, but before LiveWindow and
    * SmartDashboard integrated updating.
    */
    @Override
    public void robotPeriodic() {
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
    public void autonomousInit() {}
    
    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}
    
    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        //LEDLights.setPatternIfNotEqual(new Breathe(new Color(1.0, 0.0, 0.0), 1.0));
        Intake.init();
        Pigeon.setRelativeForward();
    }
    
    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        SwerveDrive.run(controllerOne.getLeftX(), controllerOne.getLeftY(), controllerOne.getRightX());
        
        // Left stick changes between headless and relative control modes.
        if (controllerOne.getLeftStickButtonReleased()) {
            if (SwerveDrive.getMode() == SwerveMode.Headless) {
                SwerveDrive.setMode(SwerveMode.Relative);
            } else {
                SwerveDrive.setMode(SwerveMode.Headless);
            }
        }
        
        Intake.run(
            controllerTwo.getPOV(), //controller one dpad to control pivot
            -1, //controller two dpad to control pivot
            controllerTwo.getRightTriggerAxis(), //controller one right trigger to intake
            controllerTwo.getLeftTriggerAxis(), //controller one left trigger to outtake
            0, //controller two right trigger to intake
        0, //controller two left trigger to outtake
            controllerTwo.getXButtonPressed() //controller one or two x button to toggle claw
        );
        
        ElevFourbar.run(controllerOne.getRightY(), 
            controllerOne.getLeftY(), 
            controllerOne.getStartButton(), 
            false, 
            false, 
            -1, 
            controllerOne.getRightBumperPressed(), 
            controllerOne.getAButtonPressed(), 
            controllerOne.getBButtonPressed(), 
            controllerOne.getYButtonPressed(), 
            controllerOne.getXButtonPressed(), 
            controllerOne.getLeftBumperPressed()
        );
    }
    
    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        //LEDLights.setPatternIfNotEqual(new Breathe(new Color(0.0, 1.0, 0.0), 1.0));
    }
    
    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {}
    
    /** This function is called once when test mode is enabled. */
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
