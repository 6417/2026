package frc.robot.sim;

/**
 * Final result of one simulated shot.
 */
public record ShotResult(
        boolean hit,
        double closestDistanceMeters,
        double horizontalErrorMeters,
        double verticalErrorMeters,
        double timeOfClosestApproachSec,
        double flightTimeSec) {
}
