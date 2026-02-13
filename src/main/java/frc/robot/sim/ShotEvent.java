package frc.robot.sim;

import edu.wpi.first.math.geometry.Translation3d;

/**
 * Immutable launch snapshot used by the ballistic simulator.
 *
 * @param timestampSec shot trigger timestamp from simulation clock
 * @param initialPositionField launch position in field coordinates (meters)
 * @param initialVelocityField launch velocity in field coordinates (m/s)
 * @param turretYawRad turret yaw relative to robot heading at launch time
 * @param topRpm top wheel RPM at launch
 * @param bottomRpm bottom wheel RPM at launch
 */
public record ShotEvent(
        double timestampSec,
        Translation3d initialPositionField,
        Translation3d initialVelocityField,
        double turretYawRad,
        double topRpm,
        double bottomRpm) {
}
