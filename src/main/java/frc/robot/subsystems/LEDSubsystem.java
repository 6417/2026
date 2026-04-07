package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Time;
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
import frc.robot.commands.leds.SetRainbowFullGradientCommand;
import frc.robot.commands.leds.ShootingLedsCommand;
import frc.robot.commands.shooter.ShootCommand;

public class LEDSubsystem extends SubsystemBase {
    public enum LEDMode {
        SHOOTING_NOT_READY,
        SHOOT_READY,
        SHOOTING,
        VISION_DISABLED,
        CLIMB_LATCHED,
        RAINBOWFULLGRADIENT,
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

    private final AddressableLED ledStrip;
    public final AddressableLEDBuffer ledBuffer;
    private LEDPattern ledsPattern;

    private Optional<RGB> manualOverride = Optional.empty();
    private LEDMode activeMode = LEDMode.RAINBOWFULLGRADIENT;

    private SetRainbowFullGradientCommand setRainbowFullGradientCommand;
    private ShootingLedsCommand shootingLedsCommand = new ShootingLedsCommand();

    public LEDSubsystem() {
        setRainbowFullGradientCommand = new SetRainbowFullGradientCommand();
        shootingLedsCommand = new ShootingLedsCommand();

        ledStrip = new AddressableLED(Constants.LEDs.ledPort);
        ledBuffer = new AddressableLEDBuffer(Constants.LEDs.ledBufferLength);
        ledsPattern = LEDPattern.steps(Map.of(0.0, Color.kBeige, 0.5, Color.kBrown));
        ledStrip.setLength(ledBuffer.getLength());
        setAll(OFF);
        ledStrip.setData(ledBuffer);
        ledStrip.start();
        ledStrip.setColorOrder(AddressableLED.ColorOrder.kRGB);
        setAll(RED);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("LEDs/Mode", activeMode.name());
        synchronizeLEDsWithRobotState();
        ledStrip.setData(ledBuffer);

    }

    public void setActiveMode(LEDMode mode) {
        activeMode = mode;
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
    }

    public LEDMode getActiveMode() {
        return activeMode;
    }

    public void synchronizeLEDsWithRobotState() {
        if (Math.abs(RobotContainer.shooter.getTopRpm()) > 100) {
            activeMode = LEDMode.SHOOTING;
            if (!shootingLedsCommand.isScheduled()) {
                shootingLedsCommand.schedule();
            }
            return;
        }

        if (RobotContainer.climber != null && RobotContainer.climber.isHatchetEngaged
                && RobotContainer.climber.isClimberAtPosition(RobotContainer.climber.climbPosition)) {
            activeMode = LEDMode.CLIMB_LATCHED;
            setAllianceColor();
            return;
        }

        if (RobotContainer.vision != null && !RobotContainer.vision.isVisionFusionEnabled()) {
            activeMode = LEDMode.VISION_DISABLED;
            setBlinking(ORANGE, OFF, 2.0);
            return;
        }

        if (RobotContainer.turret.isAtSetpoint()) {
            activeMode = LEDMode.SHOOT_READY;
            setAll(GREEN);
            return;
        } else {
            activeMode = LEDMode.SHOOTING_NOT_READY;
            setAll(RED);
        }

    }

    private void setAllianceColor() { // By AI
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        RGB primary = alliance == Alliance.Red ? RED : BLUE;
        setAll(primary);
    }

    public void setAllColor(int red, int green, int blue) {
        setAll(new RGB(red, green, blue));
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
    }

    private void setAllScaled(RGB color) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            setScaledRgb(i, color, 0.25);
        }
    }

    private void setScaledRgb(int index, RGB color, double brightnessScale) {
        int red = (int) Math.round(color.red * brightnessScale);
        int green = (int) Math.round(color.green * brightnessScale);
        int blue = (int) Math.round(color.blue * brightnessScale);
        ledBuffer.setRGB(index, red, green, blue);
    }
}
