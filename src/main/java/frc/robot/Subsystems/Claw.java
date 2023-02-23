package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Claw {
    private DoubleSolenoid clawSolenoid;

    public Claw(PneumaticsModuleType moduleType, int forwardChannel, int reverseChannel) {
        clawSolenoid = new DoubleSolenoid(moduleType, forwardChannel, reverseChannel);
    }

    public void setClaw(Value value) {
        clawSolenoid.set(value);
    }
}
