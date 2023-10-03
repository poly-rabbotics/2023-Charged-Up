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

    //Open the claw
    public void open() {
        clawSolenoid.set(Value.kForward);
    }

    //Close the claw
    public void close() {
        clawSolenoid.set(Value.kReverse);
    }

    //Returns the SolenoidState of the claw piston
    public SolenoidState getState() {
        if(clawSolenoid.get() == Value.kForward) {
            return SolenoidState.OPEN;
        } else {
            return SolenoidState.CLOSED;
        }
    }
}
