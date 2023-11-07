package frc.robot.systems;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Robot;
import frc.robot.Robot.ControlMode;
import frc.robot.patterns.Rainbow;
import frc.robot.patterns.BensPattern;
import frc.robot.patterns.FadeIn;
import frc.robot.patterns.FadeIntoPattern;
import frc.robot.subsystems.LightPattern;
import frc.robot.subsystems.LightRenderer;
import frc.robot.subsystems.SwerveMode;
import frc.robot.systems.AutoBalance.Stage;
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
        LightPattern setPattern = null;

        if (Robot.getControlMode() == ControlMode.DISABLED) {
            // Rainbow if disabled.
            //setPattern = new Rainbow();
            //setPattern = new FadeIntoPattern(new Rainbow(), 0.15);
            setPattern = new BensPattern(new Color(0.0, 1.0, 0.0), 1);
        } else if (Robot.getControlMode() == ControlMode.AUTONOMOUS) {
            setPattern = new FadeIn(new Color(0.0, 1.0, 0.0), 1.0);

            if (AutoBalance.getStage() == Stage.RAMMING) {
                setPattern = new FadeIn(new Color(1.0, 0.15, 0.15), 1.0);
            } else if (AutoBalance.getStage() == Stage.ADJUSTING) {
                setPattern = new FadeIn(new Color(0.0, 0.75, 0.75), 1.0);
            } else if (AutoBalance.getStage() == Stage.PAUSED) {
                setPattern = new FadeIn(new Color(0.0, 1.0, 0.0), 1.0);
            }
        } else {
            if (SwerveDrive.getDisplayMode() == SwerveMode.ROCK) {
                // If in rock mode make wyvern scary >:D
                setPattern = new FadeIn(new Color(1.0, 0.0, 0.0), 1.0);
            } else {
                // When in telop make the color correlate with game peice.
                if (ElevFourbar.getGamePieceSelected() == GamePiece.CONE) {
                    setPattern = new FadeIn(new Color(1.0, 0.5, 0.0), 1.0);
                } else if (ElevFourbar.getGamePieceSelected() == GamePiece.CUBE) {
                    setPattern = new FadeIn(new Color(1.0, 0.0, 1.0), 1.0);
                }
            }
        }

        if (setPattern != null) {
            instance.renderer.setIfNotEqual(setPattern);
        }

        instance.renderer.run();
    }
}