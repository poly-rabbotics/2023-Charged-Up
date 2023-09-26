package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SmartPrintable;

public class Roller extends SmartPrintable {
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

    @Override
    public void print() {
        SmartDashboard.putNumber("Roller Amps", pdh.getCurrent(17));
        SmartDashboard.putNumber("Roller Speed", rollerMotor.get());
    }
}
