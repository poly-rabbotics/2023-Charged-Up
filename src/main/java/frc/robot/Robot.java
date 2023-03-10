// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Fourbar;
import frc.robot.subsystems.SwerveMode;
import frc.robot.systems.ElevFourbar;
import frc.robot.systems.Intake;
import frc.robot.systems.Pigeon;
import frc.robot.systems.SwerveDrive;
import frc.robot.systems.ElevFourbar.Setpoint;
import frc.robot.systems.Intake.SolenoidState;
import edu.wpi.first.wpilibj.Timer;

/**
* The VM is configured to automatically run this class, and to call the functions corresponding to
* each mode, as described in the TimedRobot documentation. If you change the name of this class or
* the package after creating this project, you must also update the build.gradle file in the
* project.
*/
public class Robot extends TimedRobot {
    public static XboxController controllerOne = new XboxController(0);
    public static XboxController controllerTwo = new XboxController(1);
    Timer timer = new Timer();
    boolean highScoring = false;
    boolean midScoring = false;
    boolean stowed = false;
    
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
    public void robotPeriodic() {}
    
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
        ElevFourbar.init();
        Intake.comp.enableDigital();
        
        timer.reset();
        timer.start();
    }
    
    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        if(timer.get() < 5) {
            if(ElevFourbar.autoRun(Setpoint.HIGH_SCORING)) {
                Intake.autoClaw(SolenoidState.OPEN);
            }
            Intake.autoPivot(SolenoidState.UP);
        } else if(timer.get() > 5) {
            ElevFourbar.autoRun(Setpoint.STOWED);
            Intake.autoClaw(SolenoidState.CLOSED);
        }
    }
    
    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        ElevFourbar.init();
        Intake.init();
        Intake.comp.enableDigital();
        highScoring = false;
        midScoring = false;
        stowed = false;
    }
    
    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        
        ElevFourbar.run(
        controllerOne.getRightY(),
        controllerOne.getLeftY(),
        controllerOne.getStartButtonPressed(),
        controllerOne.getBackButtonPressed(),
        controllerOne.getRightBumperPressed(),
        controllerOne.getPOV(),
        false,
        controllerOne.getAButton(),
        controllerOne.getBButton(),
        controllerOne.getYButton(),
        controllerOne.getXButton(),
        controllerOne.getLeftBumperPressed()
        ); 
        
        Intake.run(
        controllerTwo.getPOV(),
        -1,
        controllerTwo.getRightTriggerAxis(),
        controllerTwo.getLeftTriggerAxis(),
        0,
        0, 
        controllerTwo.getXButtonPressed()
        );
        
    }
    
    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {}
    
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