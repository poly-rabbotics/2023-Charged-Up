package frc.robot.systems;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.patterns.Breathe;
import frc.robot.patterns.Rainbow;
import frc.robot.subsystems.LightPattern;
import frc.robot.subsystems.LightRenderer;

public class LEDLights {
    private static final int TOTAL_LENGTH = 108;
    private static final int VERTICAL_LENGTH = 68;
    private static final int UNDERGLOW_LENGTH = TOTAL_LENGTH - VERTICAL_LENGTH;

    private static final int LED_PORT = 1;
    private static final int LED_LENGTH = 69;

    private static final LightPattern[] DEFAULT_PATTERNS = {
        new Rainbow(VERTICAL_LENGTH, 50),
        new Breathe(new Color(0.0, 1.0, 0.0), UNDERGLOW_LENGTH)
    };

    private static final int[] DEFAULT_BREAKS = {
        VERTICAL_LENGTH,
        TOTAL_LENGTH
    };

    private static final LEDLights instance = new LEDLights();

    private final LightRenderer renderer;
    //private final ScheduledExecutorService executorService;

    private LEDLights() {
        

        renderer = new LightRenderer(LED_PORT, LED_LENGTH, DEFAULT_PATTERNS, DEFAULT_BREAKS);

        //executorService = Executors.newSingleThreadScheduledExecutor();
        //executorService.scheduleAtFixedRate(renderer, 0, 20, TimeUnit.MILLISECONDS);
    }

    public static void setPatternIfNotEqual(LightPattern pattern) {
        LightPattern[] patterns = {pattern, DEFAULT_PATTERNS[1] };
        instance.renderer.setIfNotEqual(patterns);
    }

    public static void run() {
        instance.renderer.run();
    }
}