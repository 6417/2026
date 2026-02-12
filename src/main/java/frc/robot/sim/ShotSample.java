package frc.robot.sim;

import edu.wpi.first.math.geometry.Translation3d;

/**
 * One sampled point of a simulated ball flight path.
 */
public record ShotSample(
        double timeSec,
        Translation3d positionField) {
}
