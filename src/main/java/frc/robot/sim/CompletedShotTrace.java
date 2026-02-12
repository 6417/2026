package frc.robot.sim;

import java.util.List;

/**
 * Full sampled trajectory and final result for one simulated shot.
 */
public record CompletedShotTrace(
        List<ShotSample> samples,
        ShotResult result) {
}
