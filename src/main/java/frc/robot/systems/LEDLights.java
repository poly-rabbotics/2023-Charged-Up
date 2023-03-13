package frc.robot.systems;

import frc.robot.patterns.Rainbow;
import frc.robot.subsystems.LightPattern;
import frc.robot.subsystems.LightRenderer;

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

        //executorService = Executors.newSingleThreadScheduledExecutor();
        //executorService.scheduleAtFixedRate(renderer, 0, 20, TimeUnit.MILLISECONDS);
    }

    public static void setPatternIfNotEqual(LightPattern pattern) {
        instance.renderer.setIfNotEqual(pattern);
    }

    public static void run() {
        instance.renderer.run();
    }
}