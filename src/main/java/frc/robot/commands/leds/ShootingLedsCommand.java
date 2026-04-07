package frc.robot.commands.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import java.util.Map;

public class ShootingLedsCommand extends SetRainbowFullGradient {

    boolean makeAShootAnimation;
    AddressableLEDBuffer ledBuffer = RobotContainer.leds.ledBuffer;
    Map<Double, Integer> shootingAnimationKeyframes;
    double animationCompletionPercentage = 0;

    @Override
    public void initialize() {
        makeAShootAnimation = false;
        shootingAnimationKeyframes = Constants.LEDs.shootingAnimationKeyframes;
        System.out.println("Shooting LEDs Activated");

    }

    @Override
    public void execute() {
        if (RobotContainer.shooter.getTopRpm() < RobotContainer.calculationSubsystem.getRPMShooter().getFirst() * 0.9) {
            makeAShootAnimation = true;
        }
        if (makeAShootAnimation) {
            animationCompletionPercentage += 0.01; // Adjust the increment value as needed
            if (animationCompletionPercentage > 1) {
                animationCompletionPercentage = 0;
                makeAShootAnimation = false;
            }
            Integer colorValue = shootingAnimationKeyframes.get(animationCompletionPercentage);
            int redValue = Color.unpackRGB(colorValue, Color.RGBChannel.kRed);
            int greenValue = Color.unpackRGB(colorValue, Color.RGBChannel.kGreen);
            int blueValue = Color.unpackRGB(colorValue, Color.RGBChannel.kBlue);
            RobotContainer.leds.setAllColor(redValue, greenValue, blueValue);
        }

    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Shooting LEDs Deactivated");
    }
}