package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Pivot {
    private final DoubleSolenoid pivotSolenoid;

    public Pivot(int forwardChannel, int reverseChannel) {
        pivotSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, forwardChannel, reverseChannel);
    }

    public void up() {
        pivotSolenoid.set(Value.kReverse);
    }

    public void down() {
        pivotSolenoid.set(Value.kForward);
    }
}
