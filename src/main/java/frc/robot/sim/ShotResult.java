package frc.robot.sim;

/**
 * Final result of one simulated shot.
 *
 * @param hit true if the ball enters the hub acceptance window before expiry
 * @param closestDistanceMeters minimum 3D center distance to hub during flight
 * @param horizontalErrorMeters horizontal distance to hub center at closest approach
 * @param verticalErrorMeters vertical offset to hub center at closest approach
 * @param timeOfClosestApproachSec simulation time of closest approach
 * @param flightTimeSec total simulated lifetime until hit or expiration
 */
public record ShotResult(
        boolean hit,
        double closestDistanceMeters,
        double horizontalErrorMeters,
        double verticalErrorMeters,
        double timeOfClosestApproachSec,
        double flightTimeSec) {
}
