package frc.robot.sim;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Shared metrics builder for focus-range shot tuning.
 */
public final class ShotFocusMetrics {
    public static final double FOCUS_MIN_DISTANCE_M = 2.0;
    public static final double FOCUS_MAX_DISTANCE_M = 5.0;
    public static final double LONG_MIN_DISTANCE_M = 5.0;
    public static final double LONG_MAX_DISTANCE_M = 6.0;

    public record ScenarioPoint(
            ShotScenarioEvaluator.Scenario scenario,
            double distanceMeters,
            double speedMps) {
    }

    public record Metrics(
            int samples,
            int focusSamples,
            int longSamples,
            double hitRate,
            double meanClosestErrorMeters,
            double saturationRate,
            double invalidRate,
            double focusHitRate,
            double focusMeanClosestErrorMeters,
            double focusSaturationRate,
            double focusInvalidRate,
            double longHitRate,
            double longMeanClosestErrorMeters,
            double longSaturationRate,
            double longInvalidRate,
            double p95ClosestErrorMeters) {
        public double objective() {
            double score = 0.0;
            score += focusHitRate * 12000.0;
            score += hitRate * 2200.0;
            score += longHitRate * 1800.0;
            score -= focusMeanClosestErrorMeters * 1400.0;
            score -= meanClosestErrorMeters * 220.0;
            score -= longMeanClosestErrorMeters * 300.0;
            score -= focusSaturationRate * 2400.0;
            score -= saturationRate * 600.0;
            score -= longSaturationRate * 700.0;
            score -= focusInvalidRate * 4200.0;
            score -= invalidRate * 1200.0;
            score -= longInvalidRate * 1500.0;
            score -= p95ClosestErrorMeters * 350.0;

            // Hard-gate style penalties for focus range.
            if (focusHitRate < 0.90) {
                score -= (0.90 - focusHitRate) * 28000.0;
            }
            if (focusSaturationRate > 0.05) {
                score -= (focusSaturationRate - 0.05) * 9000.0;
            }
            if (focusInvalidRate > 0.02) {
                score -= (focusInvalidRate - 0.02) * 14000.0;
            }
            return score;
        }
    }

    private ShotFocusMetrics() {
    }

    public static List<ScenarioPoint> buildScenarioSet(
            double[] distancesMeters,
            double[] bearingsDeg,
            double[] speedsMps,
            int directionPoints,
            Translation2d hubPositionField,
            Translation2d shooterOffsetRobot) {
        List<ScenarioPoint> list = new ArrayList<>();
        int idx = 0;
        for (double distance : distancesMeters) {
            for (double bearingDeg : bearingsDeg) {
                Rotation2d bearing = Rotation2d.fromDegrees(bearingDeg);
                Translation2d shooterPos = hubPositionField.minus(new Translation2d(distance, bearing));
                Pose2d robotPose = new Pose2d(
                        shooterPos.minus(shooterOffsetRobot),
                        Rotation2d.fromDegrees(0.0));
                for (double speed : speedsMps) {
                    for (int i = 0; i < directionPoints; i++) {
                        double dirDeg = -180.0 + (360.0 * i / (directionPoints - 1));
                        Translation2d vel = new Translation2d(speed, Rotation2d.fromDegrees(dirDeg));
                        ShotScenarioEvaluator.Scenario scenario = new ShotScenarioEvaluator.Scenario(
                                "focus_" + idx++,
                                robotPose,
                                vel);
                        list.add(new ScenarioPoint(scenario, distance, speed));
                    }
                }
            }
        }
        return list;
    }

    public static Metrics evaluate(
            ShotScenarioEvaluator evaluator,
            ShotScenarioEvaluator.EvaluationConfig config,
            List<ScenarioPoint> scenarios) {
        int n = scenarios.size();
        if (n == 0) {
            return new Metrics(0, 0, 0, 0.0, Double.POSITIVE_INFINITY, 0.0, 0.0, 0.0, Double.POSITIVE_INFINITY, 0.0, 0.0,
                    0.0, Double.POSITIVE_INFINITY, 0.0, 0.0,
                    Double.POSITIVE_INFINITY);
        }

        int hits = 0;
        int saturated = 0;
        int invalid = 0;
        double sumErr = 0.0;
        int finiteErrCount = 0;
        List<Double> finiteErrors = new ArrayList<>();

        int focusSamples = 0;
        int focusHits = 0;
        int focusSaturated = 0;
        int focusInvalid = 0;
        double focusSumErr = 0.0;
        int focusFiniteErrCount = 0;
        int longSamples = 0;
        int longHits = 0;
        int longSaturated = 0;
        int longInvalid = 0;
        double longSumErr = 0.0;
        int longFiniteErrCount = 0;

        for (ScenarioPoint point : scenarios) {
            ShotScenarioEvaluator.Evaluation eval = evaluator.evaluate(point.scenario(), config);
            if (eval.hit()) {
                hits++;
            }
            if (eval.rpmSaturatedAny()) {
                saturated++;
            }
            if (eval.solveStatus() != frc.robot.utils.ShotKinematicSolver.SolveStatus.SOLVED) {
                invalid++;
            }
            if (Double.isFinite(eval.closestDistanceMeters())) {
                sumErr += eval.closestDistanceMeters();
                finiteErrCount++;
                finiteErrors.add(eval.closestDistanceMeters());
            }

            boolean inFocus = point.distanceMeters() >= FOCUS_MIN_DISTANCE_M && point.distanceMeters() <= FOCUS_MAX_DISTANCE_M;
            if (inFocus) {
                focusSamples++;
                if (eval.hit()) {
                    focusHits++;
                }
                if (eval.rpmSaturatedAny()) {
                    focusSaturated++;
                }
                if (eval.solveStatus() != frc.robot.utils.ShotKinematicSolver.SolveStatus.SOLVED) {
                    focusInvalid++;
                }
                if (Double.isFinite(eval.closestDistanceMeters())) {
                    focusSumErr += eval.closestDistanceMeters();
                    focusFiniteErrCount++;
                }
            }

            boolean inLongRange = point.distanceMeters() > LONG_MIN_DISTANCE_M && point.distanceMeters() <= LONG_MAX_DISTANCE_M;
            if (inLongRange) {
                longSamples++;
                if (eval.hit()) {
                    longHits++;
                }
                if (eval.rpmSaturatedAny()) {
                    longSaturated++;
                }
                if (eval.solveStatus() != frc.robot.utils.ShotKinematicSolver.SolveStatus.SOLVED) {
                    longInvalid++;
                }
                if (Double.isFinite(eval.closestDistanceMeters())) {
                    longSumErr += eval.closestDistanceMeters();
                    longFiniteErrCount++;
                }
            }
        }

        finiteErrors.sort(Double::compareTo);
        double p95 = finiteErrors.isEmpty()
                ? Double.POSITIVE_INFINITY
                : finiteErrors.get((int) Math.min(finiteErrors.size() - 1, Math.floor(0.95 * (finiteErrors.size() - 1))));

        double meanErr = finiteErrCount > 0 ? (sumErr / finiteErrCount) : Double.POSITIVE_INFINITY;
        double focusMeanErr = focusFiniteErrCount > 0 ? (focusSumErr / focusFiniteErrCount) : Double.POSITIVE_INFINITY;
        double longMeanErr = longFiniteErrCount > 0 ? (longSumErr / longFiniteErrCount) : Double.POSITIVE_INFINITY;

        return new Metrics(
                n,
                focusSamples,
                longSamples,
                (double) hits / n,
                meanErr,
                (double) saturated / n,
                (double) invalid / n,
                focusSamples > 0 ? (double) focusHits / focusSamples : 0.0,
                focusMeanErr,
                focusSamples > 0 ? (double) focusSaturated / focusSamples : 0.0,
                focusSamples > 0 ? (double) focusInvalid / focusSamples : 0.0,
                longSamples > 0 ? (double) longHits / longSamples : 0.0,
                longMeanErr,
                longSamples > 0 ? (double) longSaturated / longSamples : 0.0,
                longSamples > 0 ? (double) longInvalid / longSamples : 0.0,
                p95);
    }

    public static boolean passesFocusGate(
            Metrics metrics,
            double minFocusHitRate,
            double maxFocusInvalidRate,
            double maxFocusSaturationRate) {
        return metrics.focusHitRate() >= minFocusHitRate
                && metrics.focusInvalidRate() <= maxFocusInvalidRate
                && metrics.focusSaturationRate() <= maxFocusSaturationRate;
    }
}
