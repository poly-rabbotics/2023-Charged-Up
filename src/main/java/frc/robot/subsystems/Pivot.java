package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.systems.Intake.SolenoidState;

public class Pivot {
    private final DoubleSolenoid pivotSolenoid;

    public Pivot(int forwardChannel, int reverseChannel) {
        pivotSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, forwardChannel, reverseChannel);
    }

    //Set the pivot to the up position
    public void up() {
        pivotSolenoid.set(Value.kReverse);
    }

    //Set the pivot to the down position
    public void down() {
        pivotSolenoid.set(Value.kForward);
    }

    //Returns the SolenoidState of the claw piston
    public SolenoidState getState() {
        if(pivotSolenoid.get() == Value.kForward) {
            return SolenoidState.DOWN;
        } else {
            return SolenoidState.UP;
        }
    }
}
