// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveMode;
import frc.robot.systems.Elevator;
import frc.robot.systems.Fourbar;
import frc.robot.systems.Intake;
import frc.robot.systems.Pigeon;
import frc.robot.systems.SwerveDrive;
import frc.robot.commands.IntakeAuto;
import frc.robot.commands.OuttakeAuto;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static XboxController controllerOne = new XboxController(0);
    public static XboxController controllerTwo = new XboxController(1);

    private final CommandBase[] modeOne = { new IntakeAuto(0.5, 1), new OuttakeAuto(0.5, 1) };
    private CommandBase currentCommand;
    private int autoIndex = 0;
    
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
        Intake.init();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        Intake.run(
            controllerOne.getPOV(), //controller one dpad to control pivot
            controllerTwo.getPOV(), //controller two dpad to control pivot
            controllerOne.getRightTriggerAxis(), //controller one right trigger to intake
            controllerOne.getLeftTriggerAxis(), //controller one left trigger to outtake
            controllerTwo.getRightTriggerAxis(), //controller two right trigger to intake
            controllerTwo.getLeftTriggerAxis(), //controller two left trigger to outtake
            controllerOne.getXButtonReleased() || controllerTwo.getXButtonReleased() //controller one or two x button to toggle claw
        );

        //Fourbar.run(controllerTwo.getRightY(), false, false, false);
        //Elevator.run(controllerTwo.getLeftY(), false, false, false, false, false, false, 0);
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