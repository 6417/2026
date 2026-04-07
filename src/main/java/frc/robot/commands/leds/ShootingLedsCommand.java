package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDSubsystem.LEDMode;

import java.util.Map;

public class ShootingLedsCommand extends SetRainbowFullGradientCommand {

    boolean makeAShootAnimation;
    AddressableLEDBuffer ledBuffer;
    Map<Double, Integer> shootingAnimationKeyframes;
    double animationCompletionPercentage = 0;

    double lastExecutionTimestamp = 0;
    int frameIndex = 0;

    private static final double FRAME_PERIOD_S = 0.05;      // 20 FPS Ziel
    private static final double TRIGGER_EARLY_S = 0.01;     // früher triggern gegen 20ms-Loop-Jitter

    @Override
    public void initialize() {
        makeAShootAnimation = false;
        shootingAnimationKeyframes = Constants.LEDs.shootingAnimationKeyframes;
        lastExecutionTimestamp = Timer.getFPGATimestamp();
        frameIndex = 0;
        ledBuffer = RobotContainer.leds.ledBuffer;
        System.out.println("Shooting LEDs Activated");

    }

    @Override
    public void execute() {
        if (RobotContainer.shooter.getTopRpm() < RobotContainer.calculationSubsystem.getRPMShooter().getFirst() * 0.9) {
            makeAShootAnimation = true;
        }

        if (!makeAShootAnimation) return;

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
            makeAShootAnimation = false;
            return;
        }

        double key = frameIndex * 0.05;
        Integer colorValue = shootingAnimationKeyframes.get(key);
        if (colorValue == null) return; // Protection

        int redValue = Color.unpackRGB(colorValue, Color.RGBChannel.kRed);
        int greenValue = Color.unpackRGB(colorValue, Color.RGBChannel.kGreen);
        int blueValue = Color.unpackRGB(colorValue, Color.RGBChannel.kBlue);
        RobotContainer.leds.setAllColor(redValue, greenValue, blueValue);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Shooting LEDs Deactivated");
    }
    @Override
    public boolean isFinished() {
        return RobotContainer.leds.getActiveMode() != LEDMode.SHOOTING;
    }
}