package frc.robot.commands.leds;

import java.util.Map;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDSubsystem.LEDMode;

public class IntakingLedsCommand extends Command {

    Map<Double, Color> intakingAnimationKeyframes;
    double animationDuration = 0.4; // Dauer eines kompletten Durchlaufs der Animation in Sekunden
    int frameIndex;
    double lastExecutionTimestamp = 0;
    private static final double FRAME_PERIOD_S = 0.05;      // 20 FPS Ziel
    private static final double TRIGGER_EARLY_S = 0.01;     // früher triggern gegen 20ms-Loop

    @Override
    public void initialize() {
        frameIndex = 0;
        intakingAnimationKeyframes = Constants.LEDs.intakingAnimationKeyframes;
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();
        if (now - lastExecutionTimestamp >= (FRAME_PERIOD_S - TRIGGER_EARLY_S)) {
            lastExecutionTimestamp += FRAME_PERIOD_S; // nicht "now", sonst drift/jitter
            if (now - lastExecutionTimestamp > FRAME_PERIOD_S) {
                // falls mal stark verzögert: hart resync
                lastExecutionTimestamp = now;
            }
            frameIndex++;
        }

        // Beispiel: 21 Frames => 0.00, 0.05, ... 1.00
        if (frameIndex > 20) {
            frameIndex = 0;
            return;
        }

        double keyFront = Math.floor(frameIndex * 0.5) * 0.1;
        double keyBack = (Math.floor(frameIndex*0.5) * 0.1 + 0.25) % 1.0; // versetzt für "wandernden" Effekt

        Color colorFront = intakingAnimationKeyframes.get(keyFront);
        Color colorBack = intakingAnimationKeyframes.get(keyBack);

        if (colorFront == null || colorBack == null) return; // Protection

        RobotContainer.leds.setViewFrontColor(colorFront);
        RobotContainer.leds.setViewBackColor(colorBack);
        RobotContainer.leds.setAll(colorFront);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.leds.getActiveMode() != LEDMode.INTAKING;
    }
}
