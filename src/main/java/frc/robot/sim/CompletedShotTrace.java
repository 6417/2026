package frc.robot.sim;

import java.util.List;

/**
 * Full sampled trajectory and final result for one simulated shot.
 *
 * @param samples ordered trajectory samples from launch to termination
 * @param result final hit/miss metrics for this trajectory
 */
public record CompletedShotTrace(
        List<ShotSample> samples,
        ShotResult result) {
}
