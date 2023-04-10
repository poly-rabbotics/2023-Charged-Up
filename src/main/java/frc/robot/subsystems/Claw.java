package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.systems.Intake.SolenoidState;

public class Claw {
    private final DoubleSolenoid clawSolenoid;

    public Claw(PneumaticsModuleType moduleType, int forwardChannel, int reverseChannel) {
        clawSolenoid = new DoubleSolenoid(moduleType, forwardChannel, reverseChannel);
    }

    public void open() {
        clawSolenoid.set(Value.kForward);
    }

    public void close() {
        clawSolenoid.set(Value.kReverse);
    }

    public SolenoidState getState() {
        if(clawSolenoid.get() == Value.kForward) {
            return SolenoidState.OPEN;
        } else if(clawSolenoid.get() == Value.kReverse) {
            return SolenoidState.CLOSED;
        } else {
            return SolenoidState.CLOSED;
        }
    }
}
