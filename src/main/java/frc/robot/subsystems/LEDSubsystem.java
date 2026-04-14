package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Map;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.leds.IntakingLedsCommand;
import frc.robot.commands.leds.SetRainbowFullGradientCommand;
import frc.robot.commands.leds.ShootingLedsCommand;

public class LEDSubsystem extends SubsystemBase {
    public enum LEDMode {
        SHOOTING_NOT_READY,
        SHOOT_READY,
        SHOOTING,
        VISION_DISABLED,
        CLIMB_LATCHED,
        RAINBOWFULLGRADIENT,
        INTAKING
    }

    private final AddressableLED ledStrip;
    public final AddressableLEDBuffer ledBuffer;
    private LEDPattern ledsPattern;
    public final AddressableLEDBufferView ledBufferViewBack;
    public final AddressableLEDBufferView ledBufferViewFront;

    private LEDMode activeMode = LEDMode.RAINBOWFULLGRADIENT;

    private final SetRainbowFullGradientCommand setRainbowFullGradientCommand;
    private final ShootingLedsCommand shootingLedsCommand;
    private final IntakingLedsCommand intakingLedsCommand;

    public LEDSubsystem() {
        setRainbowFullGradientCommand = new SetRainbowFullGradientCommand();
        shootingLedsCommand = new ShootingLedsCommand();
        intakingLedsCommand = new IntakingLedsCommand();

        ledStrip = new AddressableLED(Constants.LEDs.ledPort);
        ledBuffer = new AddressableLEDBuffer(Constants.LEDs.ledBufferLength);
        ledBufferViewFront = new AddressableLEDBufferView(ledBuffer, 0, (ledBuffer.getLength() / 2) - 1);
        ledBufferViewBack = new AddressableLEDBufferView(ledBuffer, ledBuffer.getLength() / 2,
                ledBuffer.getLength() - 1);

        ledsPattern = LEDPattern.steps(Map.of(0.0, Color.kBeige, 0.5, Color.kBrown));
        ledStrip.setColorOrder(AddressableLED.ColorOrder.kRGB);
        ledStrip.setLength(ledBuffer.getLength());
        setAll(Color.kBlack);
        ledStrip.setData(ledBuffer);
        ledStrip.start();
        setAllianceColor();
    }

    @Override
    public void periodic() {
        Logger.recordOutput("LEDs/Mode", activeMode.name());
        synchronizeLEDsWithRobotState();
        ledStrip.setData(ledBuffer);
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

    public void setActiveMode(LEDMode mode) {
        activeMode = mode;
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
            if (!setRainbowFullGradientCommand.isScheduled()) {
                setRainbowFullGradientCommand.schedule();
            }
            return;
        }

        if (RobotContainer.intake.activeRpsSetpoint > 0) {
            activeMode = LEDMode.INTAKING;
            if (!intakingLedsCommand.isScheduled()) {
                intakingLedsCommand.schedule();
            }
            return;
        }

        if (RobotContainer.turret.isAtSetpoint()) {
            activeMode = LEDMode.SHOOT_READY;
            setAll(Color.kGreen);
            return;
        } else {
            activeMode = LEDMode.SHOOTING_NOT_READY;
            setAll(Color.kRed);
            return;
        }
    }

    private void setAllianceColor() { // By AI
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Color primary = alliance == Alliance.Red ? Color.kRed : Color.kBlue;
        setAll(primary);
    }

    public void setAll(Color color) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED(i, color);
        }
    }

    public void setViewBackColor(Color color) {
        for (int i = 0; i < ledBufferViewBack.getLength(); i++) {
            ledBufferViewBack.setLED(i, color);
        }
    }

    public void setViewFrontColor(Color color) {
        for (int i = 0; i < ledBufferViewFront.getLength(); i++) {
            ledBufferViewFront.setLED(i, color);
        }
    }
}
