package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color.RGBChannel;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDSubsystem.LEDMode;

public class SetRainbowFullGradientCommand extends Command {
        private int currentHue;

        public SetRainbowFullGradientCommand() {
            currentHue = 0;
        }

        @Override
        public void initialize() {
            System.out.println("ççççççççççççççççç Rainbow Animation Started ççççççççççççççççç");
        }

        @Override
        public void execute() {
            if (Timer.getFPGATimestamp()%1 <= 0.9) {
                ++currentHue;
                System.out.println("§§§§§§§§§§§§§§§§§§§§§§§§§ Hue Increased §§§§§§§§§§§§§§§§§§§§§§§§§");
                if (currentHue >= 180) {
                    currentHue = 0;
                }
            }
            for (int i = 0; i < RobotContainer.leds.ledBuffer.getLength(); i++) {
                RobotContainer.leds.ledBuffer.setRGB(i,
                        Color.unpackRGB(Color.hsvToRgb(currentHue, 255, 255), RGBChannel.kRed),
                        Color.unpackRGB(Color.hsvToRgb(currentHue, 255, 255), RGBChannel.kGreen),
                        Color.unpackRGB(Color.hsvToRgb(currentHue, 255, 255), RGBChannel.kBlue));
            }
        }

        @Override
        public void end(boolean interrupted) {
        }

        @Override
        public boolean isFinished() {
            return RobotContainer.leds.getActiveMode() != LEDMode.RAINBOWFULLGRADIENT;
        }

    }
