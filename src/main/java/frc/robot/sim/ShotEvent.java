package frc.robot.sim;

import edu.wpi.first.math.geometry.Translation3d;

/**
 * Immutable launch snapshot used by the ballistic simulator.
 */
public record ShotEvent(
        double timestampSec,
        Translation3d initialPositionField,
        Translation3d initialVelocityField,
        double turretYawRad,
        double topRpm,
        double bottomRpm) {
}
