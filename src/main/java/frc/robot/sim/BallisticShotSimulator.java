package frc.robot.sim;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Simple ballistic simulator (gravity only) for shot hit/miss validation.
 */
public class BallisticShotSimulator {
    private static final class ActiveShot {
        private double x;
        private double y;
        private double z;
        private double vx;
        private double vy;
        private double vz;
        private double t;

        private double minDistanceMeters = Double.POSITIVE_INFINITY;
        private double minHorizontalErrorMeters = Double.POSITIVE_INFINITY;
        private double minVerticalErrorMeters = Double.POSITIVE_INFINITY;
        private double timeAtMinSec = 0.0;
        private final List<ShotSample> samples = new ArrayList<>();

        private ActiveShot(ShotEvent event) {
            this.x = event.initialPositionField().getX();
            this.y = event.initialPositionField().getY();
            this.z = event.initialPositionField().getZ();
            this.vx = event.initialVelocityField().getX();
            this.vy = event.initialVelocityField().getY();
            this.vz = event.initialVelocityField().getZ();
            this.t = 0.0;
            this.samples.add(new ShotSample(
                    0.0,
                    new Translation3d(this.x, this.y, this.z)));
        }
    }

    private final Translation2d hubCenterField;
    private final double hubCenterHeightMeters;
    private final double hubRadiusMeters;
    private final double hubHeightToleranceMeters;
    private final double gravityMetersPerSec2;
    private final double maxFlightTimeSec;

    private final List<ActiveShot> activeShots = new ArrayList<>();
    private final List<ShotResult> completedShots = new ArrayList<>();
    private final List<CompletedShotTrace> completedShotTraces = new ArrayList<>();
    private Optional<ShotSample> latestActiveShotSample = Optional.empty();

    public BallisticShotSimulator(
            Translation2d hubCenterField,
            double hubCenterHeightMeters,
            double hubRadiusMeters,
            double hubHeightToleranceMeters,
            double gravityMetersPerSec2,
            double maxFlightTimeSec) {
        this.hubCenterField = hubCenterField;
        this.hubCenterHeightMeters = hubCenterHeightMeters;
        this.hubRadiusMeters = hubRadiusMeters;
        this.hubHeightToleranceMeters = hubHeightToleranceMeters;
        this.gravityMetersPerSec2 = gravityMetersPerSec2;
        this.maxFlightTimeSec = maxFlightTimeSec;
    }

    public void queueShot(ShotEvent shotEvent) {
        activeShots.add(new ActiveShot(shotEvent));
    }

    public void update(double dtSec) {
        latestActiveShotSample = Optional.empty();

        Iterator<ActiveShot> iterator = activeShots.iterator();
        while (iterator.hasNext()) {
            ActiveShot shot = iterator.next();

            // Integrate position/velocity with constant gravity.
            shot.x += shot.vx * dtSec;
            shot.y += shot.vy * dtSec;
            shot.z += shot.vz * dtSec;
            shot.vz -= gravityMetersPerSec2 * dtSec;
            shot.t += dtSec;
            ShotSample currentSample = new ShotSample(shot.t, new Translation3d(shot.x, shot.y, shot.z));
            shot.samples.add(currentSample);
            latestActiveShotSample = Optional.of(currentSample);

            // Evaluate current miss metrics relative to hub center.
            double horizontalErrorMeters = Math.hypot(shot.x - hubCenterField.getX(), shot.y - hubCenterField.getY());
            double verticalErrorMeters = Math.abs(shot.z - hubCenterHeightMeters);
            double centerDistanceMeters = Math.hypot(horizontalErrorMeters, verticalErrorMeters);

            if (centerDistanceMeters < shot.minDistanceMeters) {
                shot.minDistanceMeters = centerDistanceMeters;
                shot.minHorizontalErrorMeters = horizontalErrorMeters;
                shot.minVerticalErrorMeters = verticalErrorMeters;
                shot.timeAtMinSec = shot.t;
            }

            boolean hit = horizontalErrorMeters <= hubRadiusMeters && verticalErrorMeters <= hubHeightToleranceMeters;
            boolean expired = shot.t >= maxFlightTimeSec || shot.z < 0.0;

            if (hit || expired) {
                ShotResult shotResult = new ShotResult(
                        hit,
                        shot.minDistanceMeters,
                        shot.minHorizontalErrorMeters,
                        shot.minVerticalErrorMeters,
                        shot.timeAtMinSec,
                        shot.t);
                completedShots.add(shotResult);
                completedShotTraces.add(new CompletedShotTrace(List.copyOf(shot.samples), shotResult));
                iterator.remove();
            }
        }
    }

    public List<ShotResult> drainCompletedShots() {
        List<ShotResult> result = new ArrayList<>(completedShots);
        completedShots.clear();
        return result;
    }

    public List<CompletedShotTrace> drainCompletedShotTraces() {
        List<CompletedShotTrace> result = new ArrayList<>(completedShotTraces);
        completedShotTraces.clear();
        return result;
    }

    public Optional<ShotSample> getLatestActiveShotSample() {
        return latestActiveShotSample;
    }
}
