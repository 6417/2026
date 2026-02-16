package frc.robot.sim;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Random;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.utils.LinearInterpolationTable;
import frc.robot.utils.ShotKinematicSolver;

/**
 * Auto-tuner focused on reducing moving-shot clamp saturation while keeping hit quality.
 *
 * <p>This tuner optimizes a multi-objective score:
 * - high global hit rate,
 * - low global clamp rate,
 * - low per-cell error and fewer failing (distance,speed) cells,
 * - smooth table shapes to avoid unrealistic jagged solutions.
 */
public class MovingShotAutoTuneRunner {
    private static final double[] DISTANCE_POINTS_M = {1.5, 2.5, 3.5, 4.5, 5.5};
    private static final double EVAL_DT_SEC = 0.005;

    private static final int DEFAULT_ITERATIONS = 6000;
    private static final long DEFAULT_RNG_SEED = 20260217L;

    private record TunedScenario(
            String name,
            Pose2d robotPoseField,
            Translation2d robotVelocityFieldMps,
            double distanceMeters,
            double speedMps) {
    }

    private static final class Params {
        final double[] avgRpm = new double[DISTANCE_POINTS_M.length];
        final double[] splitRpm = new double[DISTANCE_POINTS_M.length];
        final double[] distanceScaleBias = new double[DISTANCE_POINTS_M.length];
        double rpmToMpsFactor;
        double dragCoeffPerMeter;

        Params copy() {
            Params c = new Params();
            System.arraycopy(avgRpm, 0, c.avgRpm, 0, avgRpm.length);
            System.arraycopy(splitRpm, 0, c.splitRpm, 0, splitRpm.length);
            System.arraycopy(distanceScaleBias, 0, c.distanceScaleBias, 0, distanceScaleBias.length);
            c.rpmToMpsFactor = rpmToMpsFactor;
            c.dragCoeffPerMeter = dragCoeffPerMeter;
            return c;
        }
    }

    private record EvalResult(ShotResult shotResult, boolean clamped) {
    }

    private record CellStats(int count, int hitCount, int clampedCount, double sumError, int finiteErrorCount) {
        CellStats add(boolean hit, boolean clamped, double err) {
            double sum = sumError;
            int finite = finiteErrorCount;
            if (Double.isFinite(err)) {
                sum += err;
                finite++;
            }
            return new CellStats(
                    count + 1,
                    hitCount + (hit ? 1 : 0),
                    clampedCount + (clamped ? 1 : 0),
                    sum,
                    finite);
        }

        double hitRate() {
            return count > 0 ? ((double) hitCount / count) : 0.0;
        }

        double clampRate() {
            return count > 0 ? ((double) clampedCount / count) : 0.0;
        }

        double meanErr() {
            return finiteErrorCount > 0 ? (sumError / finiteErrorCount) : Double.POSITIVE_INFINITY;
        }
    }

    private static final class Score {
        final double objective;
        final double hitRate;
        final double focusHitRate;
        final double meanClosest;
        final double focusMeanClosest;
        final double meanHorizontal;
        final double meanVertical;
        final double clampRate;
        final double focusClampRate;
        final double focusInvalidRate;
        final double cellPenalty;

        Score(
                double objective,
                double hitRate,
                double focusHitRate,
                double meanClosest,
                double focusMeanClosest,
                double meanHorizontal,
                double meanVertical,
                double clampRate,
                double focusClampRate,
                double focusInvalidRate,
                double cellPenalty) {
            this.objective = objective;
            this.hitRate = hitRate;
            this.focusHitRate = focusHitRate;
            this.meanClosest = meanClosest;
            this.focusMeanClosest = focusMeanClosest;
            this.meanHorizontal = meanHorizontal;
            this.meanVertical = meanVertical;
            this.clampRate = clampRate;
            this.focusClampRate = focusClampRate;
            this.focusInvalidRate = focusInvalidRate;
            this.cellPenalty = cellPenalty;
        }
    }

    public static void main(String[] args) {
        int iterations = parseIntArg(args, 0, DEFAULT_ITERATIONS);
        long seed = parseLongArg(args, 1, DEFAULT_RNG_SEED);

        Constants.Shooter.hubPositionField = Constants.Field.HUB_CENTER_BLUE.getTranslation();

        List<TunedScenario> scenarios = buildScenarioSet();
        Params current = initialParams();
        Score currentScore = evaluate(current, scenarios);
        Params best = current.copy();
        Score bestScore = currentScore;

        Random rng = new Random(seed);
        double avgStep = 220.0;
        double splitStep = 65.0;
        double biasStep = 0.060;
        double rpmStep = 0.00020;
        double dragStep = 0.0060;

        for (int iter = 0; iter < iterations; iter++) {
            Params candidate = current.copy();
            mutate(candidate, rng, avgStep, splitStep, biasStep, rpmStep, dragStep);
            projectAndClamp(candidate);

            Score candidateScore = evaluate(candidate, scenarios);
            double temp = 1.0 - ((double) iter / (double) iterations);
            temp = Math.max(0.03, temp);
            boolean accept = candidateScore.objective > currentScore.objective;
            if (!accept) {
                double delta = candidateScore.objective - currentScore.objective;
                double prob = Math.exp(delta / (18.0 * temp));
                accept = rng.nextDouble() < prob;
            }

            if (accept) {
                current = candidate;
                currentScore = candidateScore;
            }
            if (candidateScore.objective > bestScore.objective) {
                best = candidate;
                bestScore = candidateScore;
            }

            if ((iter + 1) % 600 == 0) {
                avgStep *= 0.90;
                splitStep *= 0.90;
                biasStep *= 0.90;
                rpmStep *= 0.90;
                dragStep *= 0.90;
                System.out.printf(
                        "iter=%d | hitRate=%.3f | focusHit=%.3f | meanClosest=%.3f | focusErr=%.3f | saturation=%.3f | focusSaturation=%.3f | focusInvalid=%.3f | penalty=%.2f | rpmToMps=%.6f | drag=%.4f%n",
                        iter + 1,
                        bestScore.hitRate,
                        bestScore.focusHitRate,
                        bestScore.meanClosest,
                        bestScore.focusMeanClosest,
                        bestScore.clampRate,
                        bestScore.focusClampRate,
                        bestScore.focusInvalidRate,
                        bestScore.cellPenalty,
                        best.rpmToMpsFactor,
                        best.dragCoeffPerMeter);
            }
        }

        System.out.println();
        System.out.printf("Used iterations=%d, seed=%d%n", iterations, seed);
        printRecommendation(best, bestScore, scenarios.size());
    }

    private static int parseIntArg(String[] args, int idx, int defaultValue) {
        if (args.length <= idx) {
            return defaultValue;
        }
        try {
            return Integer.parseInt(args[idx]);
        } catch (NumberFormatException ignored) {
            return defaultValue;
        }
    }

    private static long parseLongArg(String[] args, int idx, long defaultValue) {
        if (args.length <= idx) {
            return defaultValue;
        }
        try {
            return Long.parseLong(args[idx]);
        } catch (NumberFormatException ignored) {
            return defaultValue;
        }
    }

    private static Params initialParams() {
        Params p = new Params();

        p.avgRpm[0] = (6457.7 + 6360.1) * 0.5;
        p.avgRpm[1] = (7026.1 + 6191.7) * 0.5;
        p.avgRpm[2] = (7092.4 + 6271.4) * 0.5;
        p.avgRpm[3] = (7238.0 + 6362.0) * 0.5;
        p.avgRpm[4] = (7228.0 + 6432.0) * 0.5;

        p.splitRpm[0] = 100.0;
        p.splitRpm[1] = 850.0;
        p.splitRpm[2] = 821.0;
        p.splitRpm[3] = 876.0;
        p.splitRpm[4] = 796.0;

        for (int i = 0; i < p.distanceScaleBias.length; i++) {
            p.distanceScaleBias[i] = 1.0;
        }

        p.rpmToMpsFactor = Constants.Shooter.rpmToMpsFactor;
        p.dragCoeffPerMeter = Constants.ShooterSim.dragCoefficientPerMeter;

        projectAndClamp(p);
        return p;
    }

    private static void mutate(
            Params p,
            Random rng,
            double avgStep,
            double splitStep,
            double biasStep,
            double rpmStep,
            double dragStep) {
        int variableCount = (3 * DISTANCE_POINTS_M.length) + 2;
        int which = rng.nextInt(variableCount);
        double sign = rng.nextBoolean() ? 1.0 : -1.0;
        if (which < DISTANCE_POINTS_M.length) {
            p.avgRpm[which] += sign * avgStep * (0.35 + 0.65 * rng.nextDouble());
        } else if (which < 2 * DISTANCE_POINTS_M.length) {
            int i = which - DISTANCE_POINTS_M.length;
            p.splitRpm[i] += sign * splitStep * (0.35 + 0.65 * rng.nextDouble());
        } else if (which < 3 * DISTANCE_POINTS_M.length) {
            int i = which - (2 * DISTANCE_POINTS_M.length);
            p.distanceScaleBias[i] += sign * biasStep * (0.35 + 0.65 * rng.nextDouble());
        } else if (which == 3 * DISTANCE_POINTS_M.length) {
            p.rpmToMpsFactor += sign * rpmStep * (0.35 + 0.65 * rng.nextDouble());
        } else {
            p.dragCoeffPerMeter += sign * dragStep * (0.35 + 0.65 * rng.nextDouble());
        }
    }

    private static void projectAndClamp(Params p) {
        for (int i = 0; i < p.avgRpm.length; i++) {
            p.avgRpm[i] = MathUtil.clamp(p.avgRpm[i], 1800.0, 7600.0);
            p.splitRpm[i] = MathUtil.clamp(p.splitRpm[i], 80.0, 1200.0);
            p.distanceScaleBias[i] = MathUtil.clamp(p.distanceScaleBias[i], 0.75, 1.70);
        }
        for (int i = 1; i < p.avgRpm.length; i++) {
            p.avgRpm[i] = Math.max(p.avgRpm[i], p.avgRpm[i - 1] + 25.0);

            double maxBiasStep = 0.25;
            p.distanceScaleBias[i] = MathUtil.clamp(
                    p.distanceScaleBias[i],
                    p.distanceScaleBias[i - 1] - maxBiasStep,
                    p.distanceScaleBias[i - 1] + maxBiasStep);
        }

        p.rpmToMpsFactor = MathUtil.clamp(p.rpmToMpsFactor, 0.0020, 0.0072);
        p.dragCoeffPerMeter = MathUtil.clamp(p.dragCoeffPerMeter, 0.000, 0.12);
    }

    private static Score evaluate(Params p, List<TunedScenario> scenarios) {
        Point2D[] topPoints = new Point2D[DISTANCE_POINTS_M.length];
        Point2D[] bottomPoints = new Point2D[DISTANCE_POINTS_M.length];
        Point2D[] biasPoints = new Point2D[DISTANCE_POINTS_M.length];
        for (int i = 0; i < DISTANCE_POINTS_M.length; i++) {
            double top = p.avgRpm[i] + (p.splitRpm[i] * 0.5);
            double bottom = p.avgRpm[i] - (p.splitRpm[i] * 0.5);
            topPoints[i] = new Point2D.Double(DISTANCE_POINTS_M[i], top);
            bottomPoints[i] = new Point2D.Double(DISTANCE_POINTS_M[i], bottom);
            biasPoints[i] = new Point2D.Double(DISTANCE_POINTS_M[i], p.distanceScaleBias[i]);
        }
        LinearInterpolationTable topRpmTable = new LinearInterpolationTable(topPoints);
        LinearInterpolationTable bottomRpmTable = new LinearInterpolationTable(bottomPoints);
        LinearInterpolationTable distanceBiasTable = new LinearInterpolationTable(biasPoints);

        int hits = 0;
        int clamped = 0;
        int invalid = 0;
        double sumClosest = 0.0;
        double sumHorizontal = 0.0;
        double sumVertical = 0.0;
        int finiteCount = 0;

        int focusCount = 0;
        int focusHits = 0;
        int focusClamped = 0;
        int focusInvalid = 0;
        double focusSumClosest = 0.0;

        Map<String, CellStats> cellStats = new HashMap<>();

        for (TunedScenario scenario : scenarios) {
            EvalResult result = evaluateScenario(scenario, topRpmTable, bottomRpmTable, distanceBiasTable, p);
            if (result.shotResult().hit()) {
                hits++;
            }
            if (result.clamped()) {
                clamped++;
            }
            if (!Double.isFinite(result.shotResult().closestDistanceMeters())) {
                invalid++;
            } else {
                sumClosest += result.shotResult().closestDistanceMeters();
                sumHorizontal += result.shotResult().horizontalErrorMeters();
                sumVertical += result.shotResult().verticalErrorMeters();
                finiteCount++;
            }

            if (scenario.distanceMeters() >= 2.0 && scenario.distanceMeters() <= 5.0) {
                focusCount++;
                if (result.shotResult().hit()) {
                    focusHits++;
                }
                if (result.clamped()) {
                    focusClamped++;
                }
                if (!Double.isFinite(result.shotResult().closestDistanceMeters())) {
                    focusInvalid++;
                } else {
                    focusSumClosest += result.shotResult().closestDistanceMeters();
                }
            }

            String key = String.format(Locale.US, "%.2f|%.1f", scenario.distanceMeters(), scenario.speedMps());
            CellStats s = cellStats.getOrDefault(key, new CellStats(0, 0, 0, 0.0, 0));
            cellStats.put(key, s.add(result.shotResult().hit(), result.clamped(), result.shotResult().closestDistanceMeters()));
        }

        int n = scenarios.size();
        double hitRate = (double) hits / (double) n;
        double clampRate = (double) clamped / (double) n;
        double meanClosest = finiteCount > 0 ? (sumClosest / finiteCount) : Double.POSITIVE_INFINITY;
        double meanHorizontal = finiteCount > 0 ? (sumHorizontal / finiteCount) : Double.POSITIVE_INFINITY;
        double meanVertical = finiteCount > 0 ? (sumVertical / finiteCount) : Double.POSITIVE_INFINITY;
        double invalidRate = (double) invalid / (double) n;

        double focusHitRate = focusCount > 0 ? (double) focusHits / (double) focusCount : 0.0;
        double focusClampRate = focusCount > 0 ? (double) focusClamped / (double) focusCount : 0.0;
        double focusInvalidRate = focusCount > 0 ? (double) focusInvalid / (double) focusCount : 0.0;
        int focusFinite = Math.max(1, focusCount - focusInvalid);
        double focusMeanClosest = focusSumClosest / focusFinite;

        double cellPenalty = 0.0;
        List<Double> cellErrors = new ArrayList<>();
        for (Map.Entry<String, CellStats> e : cellStats.entrySet()) {
            String[] parts = e.getKey().split("\\|");
            double distance = Double.parseDouble(parts[0]);
            double speed = Double.parseDouble(parts[1]);
            CellStats s = e.getValue();
            double cHit = s.hitRate();
            double cClamp = s.clampRate();
            double cErr = s.meanErr();
            if (Double.isFinite(cErr)) {
                cellErrors.add(cErr);
            }

            boolean inFocus = distance >= 2.0 && distance <= 5.0;
            if (inFocus) {
                if (cHit < 0.90) {
                    cellPenalty += (0.90 - cHit) * 1600.0;
                }
                if (cClamp > 0.20) {
                    cellPenalty += (cClamp - 0.20) * 1200.0;
                }
                if (Double.isFinite(cErr) && cErr > 0.45) {
                    cellPenalty += (cErr - 0.45) * 700.0;
                }
            } else {
                if (speed <= 2.0 && cHit < 0.85) {
                    cellPenalty += (0.85 - cHit) * 550.0;
                }
                if (speed > 2.0 && cHit < 0.70) {
                    cellPenalty += (0.70 - cHit) * 450.0;
                }
            }
        }
        if (focusClampRate > 0.05) {
            cellPenalty += (focusClampRate - 0.05) * 4200.0;
        }
        if (focusInvalidRate > 0.02) {
            cellPenalty += (focusInvalidRate - 0.02) * 7000.0;
        }
        if (focusHitRate < 0.90) {
            cellPenalty += (0.90 - focusHitRate) * 14000.0;
        }
        if (clampRate > 0.30) {
            cellPenalty += (clampRate - 0.30) * 1600.0;
        }
        if (invalidRate > 0.05) {
            cellPenalty += (invalidRate - 0.05) * 2600.0;
        }

        double errMean = 0.0;
        for (double v : cellErrors) {
            errMean += v;
        }
        errMean = cellErrors.isEmpty() ? 0.0 : errMean / cellErrors.size();

        double errVar = 0.0;
        for (double v : cellErrors) {
            double d = v - errMean;
            errVar += d * d;
        }
        errVar = cellErrors.isEmpty() ? 0.0 : errVar / cellErrors.size();
        double errStd = Math.sqrt(errVar);

        double smoothPenalty = tableSmoothnessPenalty(p);

        // Objective weights intentionally emphasize "keep scoring" first, then
        // reduce clamp dependence and error variance.
        double objective = (hitRate * 2200.0)
                + (focusHitRate * 7600.0)
                - (meanClosest * 170.0)
                - (focusMeanClosest * 620.0)
                - (meanHorizontal * 90.0)
                - (meanVertical * 120.0)
                - (clampRate * 620.0)
                - (focusClampRate * 1850.0)
                - (focusInvalidRate * 3600.0)
                - (errStd * 220.0)
                - smoothPenalty
                - cellPenalty;

        return new Score(
                objective,
                hitRate,
                focusHitRate,
                meanClosest,
                focusMeanClosest,
                meanHorizontal,
                meanVertical,
                clampRate,
                focusClampRate,
                focusInvalidRate,
                cellPenalty);
    }

    private static double tableSmoothnessPenalty(Params p) {
        double penalty = 0.0;
        for (int i = 1; i < p.avgRpm.length - 1; i++) {
            double secAvg = p.avgRpm[i + 1] - (2.0 * p.avgRpm[i]) + p.avgRpm[i - 1];
            double secSplit = p.splitRpm[i + 1] - (2.0 * p.splitRpm[i]) + p.splitRpm[i - 1];
            double secBias = p.distanceScaleBias[i + 1] - (2.0 * p.distanceScaleBias[i]) + p.distanceScaleBias[i - 1];
            penalty += Math.abs(secAvg) * 0.10;
            penalty += Math.abs(secSplit) * 0.18;
            penalty += Math.abs(secBias) * 260.0;
        }
        return penalty;
    }

    private static EvalResult evaluateScenario(
            TunedScenario scenario,
            LinearInterpolationTable topRpmTable,
            LinearInterpolationTable bottomRpmTable,
            LinearInterpolationTable distanceBiasTable,
            Params p) {
        Translation2d shooterPositionField = scenario.robotPoseField().getTranslation().plus(
                Constants.Shooter.shooterOffsetRobot.rotateBy(scenario.robotPoseField().getRotation()));
        Translation2d shooterToHubField = Constants.Shooter.hubPositionField.minus(shooterPositionField);
        double distanceMeters = shooterToHubField.getNorm();

        if (distanceMeters < 1e-6) {
            return new EvalResult(
                    new ShotResult(false, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, 0.0, 0.0),
                    false);
        }

        double baseTopRpm = topRpmTable.getOutput(distanceMeters);
        double baseBottomRpm = bottomRpmTable.getOutput(distanceMeters);
        double splitRpm = baseTopRpm - baseBottomRpm;

        ShotKinematicSolver.SolveResult solveResult = ShotKinematicSolver.solve(
                new ShotKinematicSolver.SolveRequest(
                        new Translation3d(
                                shooterPositionField.getX(),
                                shooterPositionField.getY(),
                                Constants.ShooterSim.launchHeightMeters),
                        new Translation3d(
                                Constants.Shooter.hubPositionField.getX(),
                                Constants.Shooter.hubPositionField.getY(),
                                Constants.ShooterSim.hubCenterHeightMeters),
                        scenario.robotVelocityFieldMps(),
                        Constants.ShooterSim.launchPitchRad,
                        Constants.ShooterSim.gravityMetersPerSec2,
                        Constants.Shooter.solverMinFlightTimeSec,
                        Constants.Shooter.solverMaxFlightTimeSec,
                        Constants.Shooter.solverMaxIterations,
                        Constants.Shooter.solverFlightTimeToleranceSec,
                        Constants.Shooter.solverVerticalErrorToleranceMeters,
                        p.dragCoeffPerMeter,
                        Constants.Shooter.solverDragSpeedCompensationGain));
        if (!solveResult.solved()) {
            return new EvalResult(
                    new ShotResult(
                            false,
                            Double.POSITIVE_INFINITY,
                            Double.POSITIVE_INFINITY,
                            Double.POSITIVE_INFINITY,
                            0.0,
                            Constants.ShooterSim.maxFlightTimeSec),
                    true);
        }

        double requiredAvgRpm = (solveResult.requiredMuzzleSpeedMps() / p.rpmToMpsFactor)
                * distanceBiasTable.getOutput(distanceMeters);
        double requestedTopRpm = requiredAvgRpm + (splitRpm * 0.5);
        double requestedBottomRpm = requiredAvgRpm - (splitRpm * 0.5);

        // "Clamped" in this tuner now means motor-speed saturation against the tuner's
        // feasible evaluation window (same window used in projectAndClamp()).
        boolean clamped = requestedTopRpm > 7600.0
                || requestedBottomRpm > 7600.0
                || requestedTopRpm < 1800.0
                || requestedBottomRpm < 1800.0;
        double correctedTopRpm = MathUtil.clamp(requestedTopRpm, 1800.0, 7600.0);
        double correctedBottomRpm = MathUtil.clamp(requestedBottomRpm, 1800.0, 7600.0);
        double actualMuzzleSpeedMps = Math.max(0.0, ((correctedTopRpm + correctedBottomRpm) * 0.5) * p.rpmToMpsFactor);

        double horizontalSpeedMps = actualMuzzleSpeedMps * Math.cos(Constants.ShooterSim.launchPitchRad);
        double verticalSpeedMps = actualMuzzleSpeedMps * Math.sin(Constants.ShooterSim.launchPitchRad);

        Translation2d muzzleVelocityField = new Translation2d(horizontalSpeedMps, solveResult.muzzleVelocityFieldMps().getAngle());
        Translation2d initialVelocityField = scenario.robotVelocityFieldMps().plus(muzzleVelocityField);

        BallisticShotSimulator simulator = new BallisticShotSimulator(
                Constants.Shooter.hubPositionField,
                Constants.ShooterSim.hubCenterHeightMeters,
                Constants.ShooterSim.hubRadiusMeters,
                Constants.ShooterSim.hubHeightToleranceMeters,
                Constants.ShooterSim.gravityMetersPerSec2,
                Constants.ShooterSim.maxFlightTimeSec,
                p.dragCoeffPerMeter);

        simulator.queueShot(new ShotEvent(
                0.0,
                new Translation3d(
                        shooterPositionField.getX(),
                        shooterPositionField.getY(),
                        Constants.ShooterSim.launchHeightMeters),
                new Translation3d(
                        initialVelocityField.getX(),
                        initialVelocityField.getY(),
                        verticalSpeedMps),
                0.0,
                correctedTopRpm,
                correctedBottomRpm));

        double elapsed = 0.0;
        while (elapsed <= Constants.ShooterSim.maxFlightTimeSec + EVAL_DT_SEC) {
            simulator.update(EVAL_DT_SEC);
            var drained = simulator.drainCompletedShots();
            if (!drained.isEmpty()) {
                return new EvalResult(drained.get(0), clamped);
            }
            elapsed += EVAL_DT_SEC;
        }

        return new EvalResult(
                new ShotResult(
                        false,
                        Double.POSITIVE_INFINITY,
                        Double.POSITIVE_INFINITY,
                        Double.POSITIVE_INFINITY,
                        0.0,
                        Constants.ShooterSim.maxFlightTimeSec),
                clamped);
    }

    private static List<TunedScenario> buildScenarioSet() {
        Translation2d hub = Constants.Shooter.hubPositionField;
        List<TunedScenario> list = new ArrayList<>();

        double[] distances = {2.0, 3.0, 4.0, 5.0, 6.0};
        double[] bearingsDeg = {-180, -135, -90, -45, 0, 45, 90, 135};
        double[] speeds = {0.0, 1.0, 2.0, 3.0};
        double[] directionsDeg = {-180, -150, -120, -90, -60, -30, 0, 30, 60, 90, 120, 150};

        int idx = 0;
        for (double distance : distances) {
            for (double bearingDeg : bearingsDeg) {
                Rotation2d bearing = Rotation2d.fromDegrees(bearingDeg);
                Translation2d shooterPos = hub.minus(new Translation2d(distance, bearing));
                Pose2d robotPose = new Pose2d(
                        shooterPos.minus(Constants.Shooter.shooterOffsetRobot),
                        Rotation2d.fromDegrees(0.0));

                for (double speed : speeds) {
                    for (double dirDeg : directionsDeg) {
                        Translation2d vel = new Translation2d(speed, Rotation2d.fromDegrees(dirDeg));
                        list.add(new TunedScenario("s" + idx++, robotPose, vel, distance, speed));
                    }
                }
            }
        }
        return list;
    }

    private static void printRecommendation(Params p, Score score, int scenarioCount) {
        double[] top = new double[DISTANCE_POINTS_M.length];
        double[] bottom = new double[DISTANCE_POINTS_M.length];
        for (int i = 0; i < DISTANCE_POINTS_M.length; i++) {
            top[i] = p.avgRpm[i] + (p.splitRpm[i] * 0.5);
            bottom[i] = p.avgRpm[i] - (p.splitRpm[i] * 0.5);
        }

        System.out.printf("Best score over %d scenarios:%n", scenarioCount);
        System.out.printf(
                "hitRate=%.3f | focusHit=%.3f | meanClosest=%.3f m | focusMeanClosest=%.3f m | meanHorizontal=%.3f m | meanVertical=%.3f m | saturationRate=%.3f | focusSaturation=%.3f | focusInvalid=%.3f | cellPenalty=%.2f%n",
                score.hitRate,
                score.focusHitRate,
                score.meanClosest,
                score.focusMeanClosest,
                score.meanHorizontal,
                score.meanVertical,
                score.clampRate,
                score.focusClampRate,
                score.focusInvalidRate,
                score.cellPenalty);
        System.out.printf("Recommended rpmToMpsFactor = %.7f%n", p.rpmToMpsFactor);
        System.out.printf("Recommended dragCoefficientPerMeter = %.5f%n", p.dragCoeffPerMeter);

        System.out.println("Recommended top RPM points:");
        for (int i = 0; i < DISTANCE_POINTS_M.length; i++) {
            System.out.printf("  (%.1f, %.1f)%n", DISTANCE_POINTS_M[i], top[i]);
        }
        System.out.println("Recommended bottom RPM points:");
        for (int i = 0; i < DISTANCE_POINTS_M.length; i++) {
            System.out.printf("  (%.1f, %.1f)%n", DISTANCE_POINTS_M[i], bottom[i]);
        }
        System.out.println("Recommended distance-scale-bias points:");
        for (int i = 0; i < DISTANCE_POINTS_M.length; i++) {
            System.out.printf("  (%.1f, %.3f)%n", DISTANCE_POINTS_M[i], p.distanceScaleBias[i]);
        }
    }
}
