package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDSubsystem.LEDMode;

import java.util.Map;

public class ShootingLedsCommand extends Command {

    boolean makeAShootAnimation;
    Map<Double, Color> shootingAnimationKeyframes;

    double lastShotTimestamp = -999;

    private boolean wasBelowShotThreshold = false;
    private double brightness = 0.0; // 0..1
    private double frameIndex = 0.0;

    private static final double LOOP_TIME_S = 0.02; // 20ms angenommen
    private static final double SHOT_THRESHOLD_FACTOR = 0.95;
    private static final double MIN_SHOT_GAP_S = 0.12; // entprellt Schuss-Erkennung
    private static final double BRIGHTNESS_DECAY_PER_S = 0.55; // 1.0 => 100% -> 0% in ~1s
    private static final double BRIGHTNESS_DECAY_PER_LOOP = 1/BRIGHTNESS_DECAY_PER_S * LOOP_TIME_S;

    @Override
    public void initialize() {
        makeAShootAnimation = false;
        shootingAnimationKeyframes = Constants.LEDs.shootingAnimationKeyframes;
        brightness = 0.0;
        frameIndex = 0.0;
        wasBelowShotThreshold = false;
        System.out.println("Shooting LEDs Activated");
    }

    @Override
    public void execute() {
        double now = Timer.getFPGATimestamp();

        double setpoint = RobotContainer.shooter.getTopSetpointRpm();
        boolean belowThreshold = setpoint > 0
                && RobotContainer.shooter.getTopRpm() < setpoint * SHOT_THRESHOLD_FACTOR;

        boolean shotEvent = belowThreshold
                && !wasBelowShotThreshold
                && (now - lastShotTimestamp) >= MIN_SHOT_GAP_S;

        wasBelowShotThreshold = belowThreshold;

        if (shotEvent) {
            makeAShootAnimation = true;
            brightness = 1.0; // sofort 100%
            lastShotTimestamp = now;
            frameIndex = 0.0;
        }

        if (!makeAShootAnimation) return;

        // feste Reduktion pro Loop (20ms angenommen)
        brightness = clamp01(brightness - BRIGHTNESS_DECAY_PER_LOOP);
        if (brightness <= 0.0) {
            makeAShootAnimation = false;
            return;
        }

        // Kein Frame-Feature mehr: feste Basisfarbe aus Keyframe-Map
        frameIndex += 0.02; // Inkrement pro Loop, damit Animation "fortschreitet" (kann auch mit Zeit statt Frames gemacht werden)
        if (frameIndex > 1.0) {
            frameIndex = 0.0; // Loop zurücksetzen, damit Animation wieder von vorne beginnt
        }
        Color baseColor = shootingAnimationKeyframes.get(0.0);
        if (baseColor == null && !shootingAnimationKeyframes.isEmpty()) {
            baseColor = shootingAnimationKeyframes.values().iterator().next();
        }
        if (baseColor == null) return;

        Color fullBrightnessColor = normalizeToFullBrightness(baseColor);
        Color out = new Color(
                clamp01(fullBrightnessColor.red * brightness),
                clamp01(fullBrightnessColor.green * brightness),
                clamp01(fullBrightnessColor.blue * brightness)
        );
        RobotContainer.leds.setAll(out);
    }

    private static Color normalizeToFullBrightness(Color c) {
        double max = Math.max(c.red, Math.max(c.green, c.blue));
        if (max <= 1e-9) return Color.kBlack;
        return new Color(c.red / max, c.green / max, c.blue / max);
    }

    private static double clamp01(double x) {
        return Math.max(0.0, Math.min(1.0, x));
    }

    @Override
    public void end(boolean interrupted) {
        makeAShootAnimation = false;
        brightness = 0.0;
        System.out.println("Shooting LEDs Deactivated");
    }

    @Override
    public boolean isFinished() {
        return RobotContainer.leds.getActiveMode() != LEDMode.SHOOTING;
    }
}