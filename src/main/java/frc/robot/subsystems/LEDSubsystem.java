package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color.RGBChannel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LEDSubsystem extends SubsystemBase {
    public enum LEDMode {
        ALLIANCE_IDLE,
        BALL_STAGED,
        SHOOT_READY,
        VISION_DISABLED,
        CLIMB_LATCHED,
        RAINBOWFULLGRADIENT,
        MANUAL
    }

    private static class RGB {
        final int red;
        final int green;
        final int blue;

        RGB(int red, int green, int blue) {
            this.red = red;
            this.green = green;
            this.blue = blue;
        }
    }

    private static final RGB OFF = new RGB(0, 0, 0);
    private static final RGB BLUE = new RGB(0, 0, 255);
    private static final RGB RED = new RGB(255, 0, 0);
    private static final RGB ORANGE = new RGB(255, 90, 0);
    private static final RGB GREEN = new RGB(0, 255, 0);
    private static final RGB WHITE = new RGB(255, 255, 255);
    
    private final AddressableLED leds;
    private final AddressableLEDBuffer ledBuffer;
    private LEDPattern ledsPattern;
    
    private Optional<RGB> manualOverride = Optional.empty();
    private LEDMode activeMode = LEDMode.ALLIANCE_IDLE;

    private SetRainbowFullGradient setRainbowFullGradientCommand;
    
    public LEDSubsystem() {
        setRainbowFullGradientCommand = new SetRainbowFullGradient();
        leds = new AddressableLED(Constants.LEDs.ledPort);
        ledBuffer = new AddressableLEDBuffer(Constants.LEDs.ledBufferLength);
        ledsPattern = LEDPattern.steps(Map.of(0.0, Color.kBeige, 0.5, Color.kBrown));
        leds.setLength(ledBuffer.getLength());
        setAll(OFF);
        leds.setData(ledBuffer);
        leds.start();
        leds.setColorOrder(AddressableLED.ColorOrder.kRGB);
        setAll(RED);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("LEDs/Mode", activeMode.name());
        ledsPattern.applyTo(ledBuffer);
        leds.setData(ledBuffer);

    }
    public void setActiveMode(LEDMode mode) {
        activeMode = mode;
            if (activeMode == LEDMode.RAINBOWFULLGRADIENT) {
                setRainbowFullGradientCommand.schedule();
            }
    }

    /**
     * Rainbow! =)
     * 
     * @param saturation  HUE Saturation of the colors
     * @param brightness  Brightness of the effect
     * @param scrollSpeed Speed in Meters per second of the rainbow effect.
     */
    public void setRainbowPattern(int saturation, int brightness) {
        ledsPattern = LEDPattern.rainbow(saturation, brightness).scrollAtAbsoluteSpeed(MetersPerSecond.of(1),
                Constants.LEDs.ledsSpacing);
        ledsPattern.applyTo(ledBuffer);
        leds.setData(ledBuffer);
    }

    public class SetRainbowFullGradient extends Command {
        private int currentHue;
        private Color currentColor;

        SetRainbowFullGradient() {
            currentHue = 0;
            currentColor = new Color();
        }

        @Override
        public void execute() {
            ++currentHue;
            if (currentHue >= 180) {
                currentHue = 0;
            }
            RobotContainer.leds.setManualColor(
                    Color.unpackRGB(Color.hsvToRgb(currentHue, 255, 255), RGBChannel.kRed),
                    Color.unpackRGB(Color.hsvToRgb(currentHue, 255, 255), RGBChannel.kGreen),
                    Color.unpackRGB(Color.hsvToRgb(currentHue, 255, 255), RGBChannel.kBlue));
        }

        @Override
        public void end(boolean interrupted) {
        }

        @Override
        public boolean isFinished() {
            return RobotContainer.leds.getActiveMode() != LEDMode.RAINBOWFULLGRADIENT;
        }

    }

    public void setManualColor(int red, int green, int blue) {
        leds.setLength(ledBuffer.getLength());
        manualOverride = Optional.of(new RGB(red, green, blue));
        activeMode = LEDMode.MANUAL;
        setAllScaled(manualOverride.get());
    }

    public void clearManualColor() {
        manualOverride = Optional.empty();
    }

    public LEDMode getActiveMode() {
        return activeMode;
    }

    public void synchronizeLEDsWithRobotState() { // By AI
        if (manualOverride.isPresent()) {
            activeMode = LEDMode.MANUAL;
            setAllScaled(manualOverride.get());
            return;
        }

        // Priority matters here:
        // 1) climb latch is the most safety-critical robot state,
        // 2) vision disabled is a driver fallback state worth showing clearly,
        // 3) shooter-ready means the robot is prepared to fire,
        // 4) ball staged is useful, but less important than the states above,
        // 5) otherwise show alliance color as the default idle state.
        if (RobotContainer.climber != null && RobotContainer.climber.isHatchetEngaged) {
            activeMode = LEDMode.CLIMB_LATCHED;
            setBlinking(RED, OFF, 4.0);
            return;
        }

        if (RobotContainer.vision != null && !RobotContainer.vision.isVisionFusionEnabled()) {
            activeMode = LEDMode.VISION_DISABLED;
            setBlinking(ORANGE, OFF, 2.0);
            return;
        }

        if (RobotContainer.shooter != null
                && RobotContainer.turret != null
                && RobotContainer.calculationSubsystem != null
                && RobotContainer.shooter.isAtSetpoint()
                && RobotContainer.turret.isAtSetpoint()
                && RobotContainer.calculationSubsystem.isSpeedOkToShoot()) {
            activeMode = LEDMode.SHOOT_READY;
            setBlinking(WHITE, OFF, 6.0);
            return;
        }

        if (RobotContainer.indexer != null && RobotContainer.indexer.isBallDetected()) {
            activeMode = LEDMode.BALL_STAGED;
            setAllScaled(GREEN);
            return;
        }

        activeMode = LEDMode.ALLIANCE_IDLE;
        setAllianceIdlePattern();
    }

    private void setAllianceIdlePattern() { // By AI
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        RGB primary = alliance == Alliance.Red ? RED : BLUE;
        RGB accent = WHITE;

        // Alternating stripes make it easier to notice the robot is powered while
        // still clearly conveying alliance color.
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            RGB color = (i % 4 < 2) ? primary : accent;
            setScaledRgb(i, color, 1.0);
        }
        leds.setData(ledBuffer);
    }

    private void setBlinking(RGB onColor, RGB offColor, double frequencyHz) { // By AI
        double phase = Timer.getFPGATimestamp() * frequencyHz;
        boolean on = ((int) Math.floor(phase)) % 2 == 0;
        setAllScaled(on ? onColor : offColor);
    }

    private void setAll(RGB color) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, color.red, color.green, color.blue);
        }
        leds.setData(ledBuffer);
    }

    private void setAllScaled(RGB color) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            setScaledRgb(i, color, 0.25);
        }
        leds.setData(ledBuffer);
    }

    private void setScaledRgb(int index, RGB color, double brightnessScale) {
        int red = (int) Math.round(color.red * Constants.LEDs.brightnessScale);
        int green = (int) Math.round(color.green * Constants.LEDs.brightnessScale);
        int blue = (int) Math.round(color.blue * Constants.LEDs.brightnessScale);
        ledBuffer.setRGB(index, red, green, blue);
    }
}
