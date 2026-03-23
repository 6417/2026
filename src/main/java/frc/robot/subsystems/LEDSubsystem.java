package frc.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
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

    private Optional<RGB> manualOverride = Optional.empty();
    private LEDMode activeMode = LEDMode.ALLIANCE_IDLE;

    public LEDSubsystem() {
        leds = new AddressableLED(Constants.LEDs.ledPort);
        ledBuffer = new AddressableLEDBuffer(Constants.LEDs.ledBufferLength);

        leds.setLength(ledBuffer.getLength());
        setAll(OFF);
        leds.setData(ledBuffer);
        leds.start();
    }

    @Override
    public void periodic() {
        synchronizeLEDsWithRobotState();
        Logger.recordOutput("LEDs/Mode", activeMode.name());
    }

    public void setManualColor(int red, int green, int blue) {
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

    public void synchronizeLEDsWithRobotState() {
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

    private void setAllianceIdlePattern() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        RGB primary = alliance == Alliance.Red ? RED : BLUE;
        RGB accent = WHITE;

        // Alternating stripes make it easier to notice the robot is powered while
        // still clearly conveying alliance color.
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            RGB color = (i % 4 < 2) ? primary : accent;
            setScaledRgb(i, color);
        }
        leds.setData(ledBuffer);
    }

    private void setBlinking(RGB onColor, RGB offColor, double frequencyHz) {
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
            setScaledRgb(i, color);
        }
        leds.setData(ledBuffer);
    }

    private void setScaledRgb(int index, RGB color) {
        int red = (int) Math.round(color.red * Constants.LEDs.brightnessScale);
        int green = (int) Math.round(color.green * Constants.LEDs.brightnessScale);
        int blue = (int) Math.round(color.blue * Constants.LEDs.brightnessScale);
        ledBuffer.setRGB(index, red, green, blue);
    }
}
