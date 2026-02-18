package frc.robot.sim;

import java.awt.geom.Point2D;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.calibration.CalibrationFitter;
import frc.robot.calibration.CalibrationSample;
import frc.robot.calibration.ShotOutcome;
import frc.robot.utils.LinearInterpolationTable;
import frc.robot.utils.ShotKinematicSolver;

/**
 * Deterministic benchmark for the reality calibrator fit loop.
 *
 * <p>Goal: run repeatable "before/after" calibration cases and export metrics for tuning.
 */
public final class RealityCalibratorBenchmarkRunner {
    private static final double DT_SEC = 0.005;
    private static final int DEFAULT_ROUNDS = 3;

    private record BenchmarkCase(
            String name,
            double modelBiasScale,
            double modelBiasTilt,
            double modelTurretOffsetDeg,
            double truthTurretOffsetDeg,
            double truthRpmScale,
            double truthDragScale) {
    }

    private record ModelState(double turretOffsetRad, double[] biasValues) {
    }

    private record PlannedShot(
            boolean valid,
            double distanceToHubMeters,
            double commandedTopRpm,
            double commandedBottomRpm,
            double commandedTurretYawRad,
            ShotKinematicSolver.SolveStatus solveStatus,
            boolean rpmSaturated) {
    }

    private record ImpactResult(
            boolean hit,
            double closestDistanceMeters,
            Translation2d closestPointField) {
    }

    private record Metrics(
            int samples,
            double hitRate,
            double meanClosestErrorMeters,
            double p95ClosestErrorMeters,
            double invalidRate,
            double saturationRate) {
    }

    private record ScenarioSets(
            List<ShotFocusMetrics.ScenarioPoint> calibration,
            List<ShotFocusMetrics.ScenarioPoint> validation) {
    }

    private record CaseResult(
            BenchmarkCase benchmarkCase,
            Metrics before,
            Metrics after,
            ModelState initialState,
            ModelState tunedState,
            int roundsApplied,
            int samplesUsed) {
    }

    private RealityCalibratorBenchmarkRunner() {
    }

    public static void main(String[] args) throws IOException {
        int rounds = parseIntArg(args, 0, DEFAULT_ROUNDS);

        Constants.Shooter.hubPositionField = Constants.Field.HUB_CENTER_BLUE.getTranslation();
        ShotScenarioEvaluator evaluator = new ShotScenarioEvaluator();
        ShotScenarioEvaluator.EvaluationConfig base = ShotScenarioEvaluator.EvaluationConfig.fromConstants();

        double[] fitKnots = Arrays.copyOf(
                Constants.Calibration.biasFitDistancesMeters,
                Constants.Calibration.biasFitDistancesMeters.length);
        ScenarioSets scenarios = buildScenarioSets(base);

        List<BenchmarkCase> cases = List.of(
                new BenchmarkCase("range_under_model", 0.94, 0.00, 0.0, 0.0, 0.88, 1.00),
                new BenchmarkCase("range_over_model", 1.06, 0.12, 0.0, 0.0, 1.08, 1.00),
                new BenchmarkCase("mixed_moderate", 0.96, 0.08, 0.8, 1.8, 0.95, 1.03));

        List<CaseResult> results = new ArrayList<>();
        for (BenchmarkCase benchmarkCase : cases) {
            CaseResult result = runCase(
                    evaluator,
                    base,
                    fitKnots,
                    scenarios,
                    rounds,
                    benchmarkCase);
            results.add(result);
            System.out.printf(
                    Locale.US,
                    "%s | before hit=%.3f err=%.3f p95=%.3f -> after hit=%.3f err=%.3f p95=%.3f | rounds=%d samples=%d%n",
                    benchmarkCase.name(),
                    result.before().hitRate(),
                    result.before().meanClosestErrorMeters(),
                    result.before().p95ClosestErrorMeters(),
                    result.after().hitRate(),
                    result.after().meanClosestErrorMeters(),
                    result.after().p95ClosestErrorMeters(),
                    result.roundsApplied(),
                    result.samplesUsed());
        }

        writeOutputs(results, rounds, Path.of("build", "shot_plots"));
    }

    private static CaseResult runCase(
            ShotScenarioEvaluator evaluator,
            ShotScenarioEvaluator.EvaluationConfig base,
            double[] fitKnots,
            ScenarioSets scenarios,
            int rounds,
            BenchmarkCase benchmarkCase) {
        double[] baseBiasAtKnots = sampleBiasValues(base.distanceScaleBiasTable(), fitKnots);

        double[] modelBias = new double[fitKnots.length];
        for (int i = 0; i < fitKnots.length; i++) {
            double normalized = (fitKnots[i] - fitKnots[0]) / (fitKnots[fitKnots.length - 1] - fitKnots[0]);
            double tilt = (normalized - 0.5) * benchmarkCase.modelBiasTilt();
            modelBias[i] = baseBiasAtKnots[i] * benchmarkCase.modelBiasScale() * (1.0 + tilt);
            modelBias[i] = MathUtil.clamp(modelBias[i], Constants.Calibration.minBiasValue, Constants.Calibration.maxBiasValue);
        }

        ModelState initial = new ModelState(
                Math.toRadians(benchmarkCase.modelTurretOffsetDeg()),
                Arrays.copyOf(modelBias, modelBias.length));

        ShotScenarioEvaluator.EvaluationConfig truthConfig = copyWithOverrides(
                base,
                Math.toRadians(benchmarkCase.truthTurretOffsetDeg()),
                base.distanceScaleBiasTable(),
                base.rpmToMpsFactor() * benchmarkCase.truthRpmScale(),
                base.ballisticDragCoeffPerMeter() * benchmarkCase.truthDragScale());

        ModelState current = initial;
        ShotScenarioEvaluator.EvaluationConfig modelConfig = copyWithOverrides(
                base,
                current.turretOffsetRad(),
                toBiasTable(fitKnots, current.biasValues()),
                base.rpmToMpsFactor(),
                base.ballisticDragCoeffPerMeter());

        Metrics before = evaluateAgainstTruth(evaluator, modelConfig, truthConfig, scenarios.validation());

        int roundsApplied = 0;
        int samplesUsed = 0;

        for (int round = 0; round < rounds; round++) {
            List<CalibrationSample> samples = collectCalibrationSamples(
                    evaluator,
                    modelConfig,
                    truthConfig,
                    scenarios.calibration());
            samplesUsed += samples.size();
            if (samples.size() < Constants.Calibration.minSamplesForFit) {
                break;
            }

            CalibrationFitter.FitResult fit = CalibrationFitter.fit(
                    samples,
                    current.turretOffsetRad(),
                    current.biasValues(),
                    fitterParams(fitKnots));

            current = new ModelState(
                    fit.turretZeroOffsetRad(),
                    Arrays.copyOf(fit.biasValues(), fit.biasValues().length));
            modelConfig = copyWithOverrides(
                    base,
                    current.turretOffsetRad(),
                    toBiasTable(fitKnots, current.biasValues()),
                    base.rpmToMpsFactor(),
                    base.ballisticDragCoeffPerMeter());
            roundsApplied++;
        }

        Metrics after = evaluateAgainstTruth(evaluator, modelConfig, truthConfig, scenarios.validation());
        return new CaseResult(benchmarkCase, before, after, initial, current, roundsApplied, samplesUsed);
    }

    private static List<CalibrationSample> collectCalibrationSamples(
            ShotScenarioEvaluator evaluator,
            ShotScenarioEvaluator.EvaluationConfig modelConfig,
            ShotScenarioEvaluator.EvaluationConfig truthConfig,
            List<ShotFocusMetrics.ScenarioPoint> scenarios) {
        List<CalibrationSample> samples = new ArrayList<>();
        int idx = 0;

        for (ShotFocusMetrics.ScenarioPoint point : scenarios) {
            PlannedShot planned = planShot(evaluator, point, modelConfig);
            if (!planned.valid()) {
                continue;
            }

            Optional<ImpactResult> predictedImpact = simulateImpact(point, planned, modelConfig, 0.0);
            double yawDeltaTruth = truthConfig.turretZeroOnRobotRad() - modelConfig.turretZeroOnRobotRad();
            Optional<ImpactResult> actualImpact = simulateImpact(point, planned, truthConfig, yawDeltaTruth);
            if (predictedImpact.isEmpty() || actualImpact.isEmpty()) {
                continue;
            }

            ShotOutcome outcome = actualImpact.get().hit() ? ShotOutcome.HIT : ShotOutcome.MISS_UNKNOWN;

            CalibrationSample sample = new CalibrationSample(
                    idx++,
                    "sim-benchmark",
                    0,
                    "synthetic",
                    planned.distanceToHubMeters(),
                    point.scenario().robotPoseField(),
                    point.scenario().robotVelocityFieldMps(),
                    planned.distanceToHubMeters(),
                    planned.commandedTopRpm(),
                    planned.commandedBottomRpm(),
                    planned.commandedTurretYawRad(),
                    planned.solveStatus(),
                    planned.rpmSaturated(),
                    outcome,
                    predictedImpact.get().closestPointField(),
                    actualImpact.get().closestPointField(),
                    true,
                    1.0);
            samples.add(sample);
        }

        return samples;
    }

    private static Metrics evaluateAgainstTruth(
            ShotScenarioEvaluator evaluator,
            ShotScenarioEvaluator.EvaluationConfig modelConfig,
            ShotScenarioEvaluator.EvaluationConfig truthConfig,
            List<ShotFocusMetrics.ScenarioPoint> validationScenarios) {
        int samples = 0;
        int hits = 0;
        int invalid = 0;
        int saturated = 0;
        List<Double> closestErrors = new ArrayList<>();

        for (ShotFocusMetrics.ScenarioPoint point : validationScenarios) {
            PlannedShot planned = planShot(evaluator, point, modelConfig);
            if (!planned.valid()) {
                invalid++;
                continue;
            }
            samples++;
            if (planned.rpmSaturated()) {
                saturated++;
            }

            double yawDeltaTruth = truthConfig.turretZeroOnRobotRad() - modelConfig.turretZeroOnRobotRad();
            Optional<ImpactResult> actualImpact = simulateImpact(point, planned, truthConfig, yawDeltaTruth);
            if (actualImpact.isEmpty()) {
                invalid++;
                continue;
            }

            ImpactResult impact = actualImpact.get();
            if (impact.hit()) {
                hits++;
            }
            if (Double.isFinite(impact.closestDistanceMeters())) {
                closestErrors.add(impact.closestDistanceMeters());
            }
        }

        if (samples <= 0) {
            return new Metrics(0, 0.0, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, 1.0, 0.0);
        }

        closestErrors.sort(Comparator.naturalOrder());
        double mean = closestErrors.stream().mapToDouble(Double::doubleValue).average().orElse(Double.POSITIVE_INFINITY);
        double p95 = closestErrors.isEmpty()
                ? Double.POSITIVE_INFINITY
                : closestErrors.get((int) Math.min(
                        closestErrors.size() - 1,
                        Math.floor(0.95 * (closestErrors.size() - 1))));

        return new Metrics(
                samples,
                (double) hits / samples,
                mean,
                p95,
                (double) invalid / samples,
                (double) saturated / samples);
    }

    private static PlannedShot planShot(
            ShotScenarioEvaluator evaluator,
            ShotFocusMetrics.ScenarioPoint point,
            ShotScenarioEvaluator.EvaluationConfig config) {
        ShotScenarioEvaluator.Evaluation eval = evaluator.evaluate(point.scenario(), config);
        if (eval.solveStatus() != ShotKinematicSolver.SolveStatus.SOLVED) {
            return new PlannedShot(
                    false,
                    point.distanceMeters(),
                    0.0,
                    0.0,
                    0.0,
                    eval.solveStatus(),
                    false);
        }
        return new PlannedShot(
                true,
                point.distanceMeters(),
                eval.topRpm(),
                eval.bottomRpm(),
                Math.toRadians(eval.turretYawRobotDeg()),
                eval.solveStatus(),
                eval.rpmSaturatedAny());
    }

    private static Optional<ImpactResult> simulateImpact(
            ShotFocusMetrics.ScenarioPoint point,
            PlannedShot planned,
            ShotScenarioEvaluator.EvaluationConfig executionConfig,
            double yawOffsetRad) {
        Pose2d robotPose = point.scenario().robotPoseField();
        Translation2d robotVel = point.scenario().robotVelocityFieldMps();

        Translation2d shooterPositionField = robotPose.getTranslation()
                .plus(executionConfig.shooterOffsetRobot().rotateBy(robotPose.getRotation()));
        double yawFieldRad = robotPose.getRotation().getRadians() + planned.commandedTurretYawRad() + yawOffsetRad;

        double avgRpm = (planned.commandedTopRpm() + planned.commandedBottomRpm()) * 0.5;
        double muzzleSpeed = Math.max(0.0, avgRpm * executionConfig.rpmToMpsFactor());
        double horizontalSpeed = muzzleSpeed * Math.cos(executionConfig.launchPitchRad());
        double verticalSpeed = muzzleSpeed * Math.sin(executionConfig.launchPitchRad());

        Translation2d muzzleVelocityField = new Translation2d(horizontalSpeed, new Rotation2d(yawFieldRad));
        Translation2d initialVelocityField = robotVel.plus(muzzleVelocityField);

        BallisticShotSimulator simulator = new BallisticShotSimulator(
                executionConfig.hubPositionField(),
                executionConfig.hubCenterHeightMeters(),
                executionConfig.hubRadiusMeters(),
                executionConfig.hubHeightToleranceMeters(),
                executionConfig.gravityMetersPerSec2(),
                executionConfig.maxFlightTimeSec(),
                executionConfig.ballisticDragCoeffPerMeter());

        simulator.queueShot(new ShotEvent(
                0.0,
                new Translation3d(
                        shooterPositionField.getX(),
                        shooterPositionField.getY(),
                        executionConfig.launchHeightMeters()),
                new Translation3d(
                        initialVelocityField.getX(),
                        initialVelocityField.getY(),
                        verticalSpeed),
                planned.commandedTurretYawRad(),
                planned.commandedTopRpm(),
                planned.commandedBottomRpm()));

        Optional<CompletedShotTrace> traceOpt = Optional.empty();
        double elapsed = 0.0;
        while (elapsed <= executionConfig.maxFlightTimeSec() + DT_SEC) {
            simulator.update(DT_SEC);
            List<CompletedShotTrace> traces = simulator.drainCompletedShotTraces();
            if (!traces.isEmpty()) {
                traceOpt = Optional.of(traces.get(0));
                break;
            }
            elapsed += DT_SEC;
        }

        if (traceOpt.isEmpty()) {
            return Optional.empty();
        }

        CompletedShotTrace trace = traceOpt.get();
        Translation2d closestPoint = closestPointAtTime(trace.samples(), trace.result().timeOfClosestApproachSec());
        return Optional.of(new ImpactResult(
                trace.result().hit(),
                trace.result().closestDistanceMeters(),
                closestPoint));
    }

    private static Translation2d closestPointAtTime(List<ShotSample> samples, double targetTimeSec) {
        ShotSample best = samples.get(0);
        double bestDt = Math.abs(best.timeSec() - targetTimeSec);
        for (ShotSample sample : samples) {
            double dt = Math.abs(sample.timeSec() - targetTimeSec);
            if (dt < bestDt) {
                best = sample;
                bestDt = dt;
            }
        }
        return new Translation2d(best.positionField().getX(), best.positionField().getY());
    }

    private static CalibrationFitter.Params fitterParams(double[] fitKnots) {
        return new CalibrationFitter.Params(
                fitKnots,
                Constants.Calibration.biasWeightWindowMeters,
                Constants.Calibration.biasLearningRate,
                Constants.Calibration.maxBiasRelativeStep,
                Constants.Calibration.minBiasValue,
                Constants.Calibration.maxBiasValue,
                Constants.Calibration.minDirectionalSamplesForTurretFit,
                Constants.Calibration.turretDegPerDirectionalMiss,
                Constants.Calibration.maxTurretOffsetDeg,
                Constants.Calibration.impactResidualWeight,
                Constants.Calibration.impactRangeMetersPerBiasUnit,
                Constants.Calibration.impactLateralMetersPerTurretDeg,
                Constants.Calibration.turretResidualLearningRate,
                Constants.Calibration.biasResidualLearningRate,
                Constants.Calibration.maxTurretDeltaDegPerRound,
                Constants.Calibration.maxBiasDeltaPerRound,
                Constants.Calibration.minLateralWeightForTurretUpdate,
                Constants.Calibration.minRangeWeightForBiasUpdate,
                Constants.Calibration.fitShrinkFactorLowConfidence,
                Constants.Calibration.residualHuberDeltaMeters);
    }

    private static ScenarioSets buildScenarioSets(ShotScenarioEvaluator.EvaluationConfig base) {
        List<ShotFocusMetrics.ScenarioPoint> calibration = ShotFocusMetrics.buildScenarioSet(
                new double[] {2.5, 3.5, 4.5, 5.5},
                new double[] {-135.0, -45.0, 45.0, 135.0},
                new double[] {0.0, 1.0, 2.0},
                13,
                base.hubPositionField(),
                base.shooterOffsetRobot());

        List<ShotFocusMetrics.ScenarioPoint> validation = ShotFocusMetrics.buildScenarioSet(
                new double[] {2.0, 3.0, 4.0, 5.0, 6.0},
                new double[] {-180.0, -120.0, -60.0, 0.0, 60.0, 120.0},
                new double[] {0.0, 1.0, 2.0, 3.0},
                17,
                base.hubPositionField(),
                base.shooterOffsetRobot());

        return new ScenarioSets(calibration, validation);
    }

    private static double[] sampleBiasValues(LinearInterpolationTable table, double[] distancesM) {
        double[] values = new double[distancesM.length];
        for (int i = 0; i < distancesM.length; i++) {
            values[i] = table.getOutput(distancesM[i]);
        }
        return values;
    }

    private static LinearInterpolationTable toBiasTable(double[] distancesM, double[] values) {
        Point2D[] points = new Point2D[distancesM.length];
        for (int i = 0; i < distancesM.length; i++) {
            points[i] = new Point2D.Double(distancesM[i], values[i]);
        }
        return new LinearInterpolationTable(points);
    }

    private static ShotScenarioEvaluator.EvaluationConfig copyWithOverrides(
            ShotScenarioEvaluator.EvaluationConfig base,
            double turretZeroOffsetRad,
            LinearInterpolationTable biasTable,
            double rpmToMpsFactor,
            double ballisticDragCoeffPerMeter) {
        return new ShotScenarioEvaluator.EvaluationConfig(
                base.hubPositionField(),
                base.shooterOffsetRobot(),
                turretZeroOffsetRad,
                base.topRpmTable(),
                base.bottomRpmTable(),
                biasTable,
                rpmToMpsFactor,
                base.maxRpm(),
                base.launchHeightMeters(),
                base.hubCenterHeightMeters(),
                base.hubRadiusMeters(),
                base.hubHeightToleranceMeters(),
                base.launchPitchRad(),
                base.gravityMetersPerSec2(),
                base.maxFlightTimeSec(),
                ballisticDragCoeffPerMeter,
                base.solverDragCoeffPerMeter(),
                base.solverDragSpeedCompensationGain(),
                base.solverMinFlightTimeSec(),
                base.solverMaxFlightTimeSec(),
                base.solverMaxIterations(),
                base.solverFlightTimeToleranceSec(),
                base.solverVerticalErrorToleranceMeters());
    }

    private static void writeOutputs(List<CaseResult> results, int rounds, Path outDir) throws IOException {
        Files.createDirectories(outDir);

        Path csvPath = outDir.resolve("reality_calibrator_benchmark.csv");
        List<String> csv = new ArrayList<>();
        csv.add("case,rounds,before_hit,after_hit,before_mean_err,after_mean_err,before_p95,after_p95,before_invalid,after_invalid,before_sat,after_sat,init_turret_deg,final_turret_deg,samples_used");
        for (CaseResult result : results) {
            csv.add(String.format(
                    Locale.US,
                    "%s,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d",
                    result.benchmarkCase().name(),
                    result.roundsApplied(),
                    result.before().hitRate(),
                    result.after().hitRate(),
                    result.before().meanClosestErrorMeters(),
                    result.after().meanClosestErrorMeters(),
                    result.before().p95ClosestErrorMeters(),
                    result.after().p95ClosestErrorMeters(),
                    result.before().invalidRate(),
                    result.after().invalidRate(),
                    result.before().saturationRate(),
                    result.after().saturationRate(),
                    Math.toDegrees(result.initialState().turretOffsetRad()),
                    Math.toDegrees(result.tunedState().turretOffsetRad()),
                    result.samplesUsed()));
        }
        Files.write(csvPath, csv, StandardCharsets.UTF_8);

        Path summaryPath = outDir.resolve("reality_calibrator_benchmark_summary.txt");
        List<String> summary = new ArrayList<>();
        summary.add(String.format(Locale.US, "reality calibrator benchmark (rounds=%d)", rounds));
        summary.add("");
        for (CaseResult result : results) {
            summary.add(String.format(Locale.US, "[%s]", result.benchmarkCase().name()));
            summary.add(String.format(
                    Locale.US,
                    "before: hit=%.3f meanErr=%.3f p95=%.3f invalid=%.3f sat=%.3f",
                    result.before().hitRate(),
                    result.before().meanClosestErrorMeters(),
                    result.before().p95ClosestErrorMeters(),
                    result.before().invalidRate(),
                    result.before().saturationRate()));
            summary.add(String.format(
                    Locale.US,
                    "after : hit=%.3f meanErr=%.3f p95=%.3f invalid=%.3f sat=%.3f",
                    result.after().hitRate(),
                    result.after().meanClosestErrorMeters(),
                    result.after().p95ClosestErrorMeters(),
                    result.after().invalidRate(),
                    result.after().saturationRate()));
            summary.add(String.format(
                    Locale.US,
                    "turret: init=%.2fdeg final=%.2fdeg | rounds=%d | samples=%d",
                    Math.toDegrees(result.initialState().turretOffsetRad()),
                    Math.toDegrees(result.tunedState().turretOffsetRad()),
                    result.roundsApplied(),
                    result.samplesUsed()));
            summary.add("");
        }
        Files.write(summaryPath, summary, StandardCharsets.UTF_8);
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
}
