// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.SwerveMode;
import frc.robot.systems.Pigeon;
import frc.robot.systems.SwerveDrive;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static XboxController testController = new XboxController(1);
    
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
    public void autonomousInit() {}

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        Pigeon.setRelativeForward();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        double leftX = testController.getLeftX();
        double leftY = testController.getLeftY();
        double speed = Math.sqrt(leftX*leftX + leftY*leftY);
        
        if (Math.abs(leftX) < 0.15 && Math.abs(leftY) < 0.15)
            speed = 0.0;
        
        speed *= speed * speed;

        SwerveDrive.run(leftX, leftY, testController.getRightX(), speed);

        if (testController.getLeftStickButtonReleased()) {
            if (SwerveDrive.getMode() == SwerveMode.Headless) {
                SwerveDrive.setMode(SwerveMode.Relative);
            } else {
                SwerveDrive.setMode(SwerveMode.Headless);
            }
        }

        if (testController.getRightBumperReleased()) {
            SwerveDrive.addToP(0.0001);
        }

        if (testController.getLeftBumperReleased()) {
            SwerveDrive.addToP(-0.0001);
        }

        if (testController.getYButtonReleased()) {
            SwerveDrive.addToI(0.0000001);
        }

        if (testController.getAButtonReleased()) {
            SwerveDrive.addToI(-0.0000001);
        }

        if (testController.getXButtonReleased()) {
            SwerveDrive.addToD(0.0000001);
        }

        if (testController.getBButtonReleased()) {
            SwerveDrive.addToD(-0.0000001);
        }

        if (testController.getStartButtonReleased()) {
            SwerveDrive.resetPos();
        }
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
