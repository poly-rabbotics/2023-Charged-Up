package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Pivot {
    private DoubleSolenoid pivotSolenoid;

    public Pivot(PneumaticsModuleType moduleType, int forwardChannel, int reverseChannel) {
        pivotSolenoid = new DoubleSolenoid(moduleType, forwardChannel, reverseChannel);
    }

    public void setPivot(Value value) {
        pivotSolenoid.set(value);
    }
}
