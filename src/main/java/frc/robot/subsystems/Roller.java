package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;

public class Roller {
    private static final int PDH_ID = 41;
    private static final double MAX_AMPS = 1000;

    private final PWMTalonSRX rollerMotor;
    private final PowerDistribution pdh;

    public Roller(int rightRollerID) {
        rollerMotor = new PWMTalonSRX(rightRollerID);
        pdh = new PowerDistribution(PDH_ID, ModuleType.kRev);
    }

    public void setSpeed(double speed) {
        if (pdh.getCurrent(rollerMotor.getChannel()) > MAX_AMPS) {
            speed = 0.1;
        } 

        rollerMotor.set(speed);
    }

    public double getAmps() {
        return pdh.getCurrent(17);
    }

    public double get() {
        return rollerMotor.get();
    }
}
