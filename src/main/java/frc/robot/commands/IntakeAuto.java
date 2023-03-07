package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.systems.Intake;
import frc.robot.systems.Intake.SolenoidState;
import edu.wpi.first.wpilibj.Timer;

public class IntakeAuto extends CommandBase {
    Timer timer = new Timer();
    double speed, duration;

    public IntakeAuto(double speed, double duration) {
        this.speed = speed;
        this.duration = duration;
    }

    public void initialize() {
        timer.reset();
        timer.start();
    }

    public void execute() {
        Intake.rollerAuto(speed);
        Intake.clawAuto(SolenoidState.CLOSED);
    }

    public void end(boolean interrupted) {
        Intake.rollerAuto(0);
    }

    public boolean isFinished() {
        return timer.get() > duration;
    }
}
