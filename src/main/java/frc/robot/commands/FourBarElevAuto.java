package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.systems.ElevFourbar;
import frc.robot.systems.ElevFourbar.Setpoint;
import frc.robot.systems.ElevFourbar;

public class FourBarElevAuto extends CommandBase {
    Setpoint setpoint;

    public FourBarElevAuto(Setpoint setpoint) {
        this.setpoint = setpoint;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        ElevFourbar.autonomousRun(setpoint);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return ElevFourbar.getIsFinished();
    }
}
