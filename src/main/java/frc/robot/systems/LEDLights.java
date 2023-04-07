package frc.robot.systems;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;
import frc.robot.Robot.ControlMode;
import frc.robot.patterns.Breathe;
import frc.robot.patterns.Rainbow;
import frc.robot.subsystems.LightRenderer;
import frc.robot.systems.ElevFourbar.GamePiece;

public class LEDLights {
    // Public since it may be usefule for pattern instantiation.
    public static final int LED_LENGTH = 108;
    private static final int LED_PORT = 1;

    private static final LEDLights instance = new LEDLights();

    private final LightRenderer renderer;
    //private final ScheduledExecutorService executorService;

    private LEDLights() {
        renderer = new LightRenderer(LED_PORT, LED_LENGTH);
        renderer.setPattern(new Rainbow(69, 100));
    }

    public static void run() {
        /**
         * Reading this you may notice that things use else blocks excessively,
         * if we were setting an int it wouldnt matter, but because we set if 
         * NOT equal to maintane the animation timer we should only make one set
         * call per loop as to not distub the timer.
         */

        if (Robot.getControlMode() == ControlMode.DISABLED) {
            // Rainbow if disabled.
            instance.renderer.setIfNotEqual(new Rainbow());
        } else if (Robot.getControlMode() == ControlMode.AUTONOMOUS) {
            // TODO: expose some information about autonomous modes, e.g. what 
            // actions they perform. This way we can set colors for auto balance
            // selectively.
        } else {
            if (SwerveDrive.getRockMode()) {
                // If in rock mode make wyvern scary >:D
                instance.renderer.setIfNotEqual(new Breathe(new Color(1.0, 0.0, 0.0), 1.5));
            } else {
                // When in telop make the color cooralate with game peice.
                if (ElevFourbar.getSelectedGamePeice() == GamePiece.CONE) {
                    instance.renderer.setIfNotEqual(new Breathe(new Color(1.0, 1.0, 0.0), 0.5));
                } else if (ElevFourbar.getSelectedGamePeice() == GamePiece.CUBE) {
                    instance.renderer.setIfNotEqual(new Breathe(new Color(1.0, 0.0, 1.0), 0.5));
                }
            }
        }

        instance.renderer.run();
    }
}