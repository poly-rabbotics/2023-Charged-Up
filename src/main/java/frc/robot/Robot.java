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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveMode;
import frc.robot.systems.ElevFourbar;
import frc.robot.systems.Intake;
import frc.robot.systems.Pigeon;
import frc.robot.systems.SwerveDrive;
import frc.robot.systems.ElevFourbar.Setpoint;
import frc.robot.systems.Intake.SolenoidState;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.systems.LEDLights;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
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
    public static AnalogInput pressureSensor = new AnalogInput(0);
    public static Joystick controlPanel = new Joystick(2);
    Timer timer = new Timer();
    
    boolean autoStageOne;
    
    /**
    * This function is run when the robot is first started up and should be used for any
    * initialization code.
    */
    @Override
    public void robotInit() {
        //Pigeon.setStartingAngle(180 * 2);
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
        SwerveDrive.print();
        double pressureValue = (pressureSensor.getValue() - 410) / 13.5;
        LEDLights.run();
        SmartDashboard.putNumber("Comp Pressure", Math.floor(pressureValue));

        SmartDashboard.putBoolean("Fully Pressurized", pressureValue > 60);
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
        Pigeon.setRelativeForward();
        LEDLights.setPatternIfNotEqual(new Breathe(new Color(0.8, 0.3, 0.0), 0.5));
        ElevFourbar.autonomousInit();
        timer.reset();
        timer.stop();
        autoStageOne = false;
        //SwerveDrive.setMode(SwerveMode.Relative);
    }
    
    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        // SwerveDrive.autoSetSetpoints(0.5, Pigeon.getRelativeRotationDegrees());
        timer.start();
        if(timer.get() < 2)
        return;

        if(controlPanel.getRawButton(12)) {
            if(!autoStageOne){
                Intake.autoPivot(SolenoidState.UP);
                if(ElevFourbar.autoRun(Setpoint.HIGH_SCORING)) {
                    Intake.autoClaw(SolenoidState.OPEN);
                    timer.start();
                    autoStageOne = true;
                } 
            } else {
                if(timer.get() > 5) {
                    if(timer.get() > 5.5) Intake.autoClaw(SolenoidState.CLOSED);
                    ElevFourbar.autoRun(Setpoint.STOWED);
                }
            }
        }

        if (timer.get() > 10 && timer.get() < 12) {
           SwerveDrive.run(0.0, -0.5, 0.0);
        } else {
            SwerveDrive.run(0.0, 0.0, 0.0);
        }
    }
    
    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        //SwerveDrive.zeroEncoders();
        Pigeon.setRelativeForward();
        SwerveDrive.setMode(SwerveMode.Headless);
        LEDLights.setPatternIfNotEqual(new Breathe(new Color(0.0, 1.0, 0.0), 0.5));
        ElevFourbar.init();
        Intake.init();
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
        controlPanel.getRawButtonPressed(8), //controller one dpad to control pivot
        controlPanel.getRawButton(9),
        controlPanel.getRawButton(7),
        controlPanel.getRawButtonPressed(6)
        );
        
        ElevFourbar.run(
        controllerTwo.getRightY(),
        controllerTwo.getLeftY(),
        controllerTwo.getStartButton(),
        false,
        false,
        controllerTwo.getPOV(),
        controlPanel.getRawButton(3),
        controlPanel.getRawButton(2),
        controlPanel.getRawButton(4),
        controlPanel.getRawButton(5),
        controlPanel.getRawButton(1),
        false
        );
    }
    
    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        LEDLights.setPatternIfNotEqual(new Rainbow(LEDLights.LED_LENGTH, 50));
    }
    
    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
    }
    
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
