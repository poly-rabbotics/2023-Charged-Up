package frc.robot.systems;
import edu.wpi.first.wpilibj.Ultrasonic;

public class Sonar {

    private static final int SONAR0_PING_CHANNEL = 0;
    private static final int SONAR0_ECHO_CHANNEL = 1;

    private static final int SONAR1_PING_CHANNEL = 2;
    private static final int SONAR1_ECHO_CHANNEL = 3;

    private Ultrasonic sonar0;
    private Ultrasonic sonar1;

    public static Sonar instance = new Sonar();

    public Sonar() {
        sonar0 = new Ultrasonic(SONAR0_PING_CHANNEL, SONAR0_ECHO_CHANNEL);
        sonar1 = new Ultrasonic(SONAR1_PING_CHANNEL, SONAR1_ECHO_CHANNEL);
    }

    /**
     * Get the average distance from the two sonars in inches
     * @return distance in inches
     */
    public static double getDistance() {
        return (instance.sonar0.getRangeInches() + instance.sonar1.getRangeInches()) / 2;
    }
}
