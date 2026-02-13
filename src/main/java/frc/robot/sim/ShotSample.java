package frc.robot.sim;

import edu.wpi.first.math.geometry.Translation3d;

/**
 * One sampled point of a simulated ball flight path.
 *
 * @param timeSec simulation time from launch
 * @param positionField ball position in field coordinates (meters)
 */
public record ShotSample(
        double timeSec,
        Translation3d positionField) {
}
