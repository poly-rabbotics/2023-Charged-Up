
package frc.robot.systems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;

public class LEDLights {
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private Timer timer = new Timer();

    private static LEDLights instance = new LEDLights();

    public static void setLED(AddressableLED led) {
        instance.led = led;
    }
    
    private LEDLights() {
        timer.start();
        ledBuffer = new AddressableLEDBuffer(10);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }
}