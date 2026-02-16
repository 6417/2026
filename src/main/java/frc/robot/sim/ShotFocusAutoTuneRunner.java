package frc.robot.sim;

import java.awt.geom.Point2D;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

import frc.robot.Constants;

/**
 * Deterministic focus-range tuner for the analytic moving-shot solver.
 *
 * <p>Targets 2-5 m first (hit-rate gate), then minimizes saturation/invalid/error.
 */
public class ShotFocusAutoTuneRunner {
    private static final double[] TUNE_DISTANCES_M = {2.0, 3.0, 4.0, 5.0, 6.0};
    private static final double[] VERIFY_DISTANCES_M = {2.0, 3.0, 4.0, 5.0, 6.0};
    private static final double[] BEARINGS_DEG = {-180.0, -135.0, -90.0, -45.0, 0.0, 45.0, 90.0, 135.0};
    private static final double[] SPEEDS_MPS = {0.0, 1.0, 2.0, 3.0};
    private static final int TUNE_DIRECTION_POINTS = 19;
    private static final int VERIFY_DIRECTION_POINTS = 37;

    // Focus gates (2-5m).
    private static final double FOCUS_GATE_HIT_RATE = 0.90;
    private static final double FOCUS_GATE_MAX_INVALID_RATE = 0.02;
    private static final double FOCUS_GATE_MAX_SATURATION_RATE = 0.05;

    private record TuneParams(
            double solverDragGain,
            double biasAt35m,
            double biasAt45m,
            double biasAt55m) {
        String key() {
            return String.format(Locale.US, "%.4f|%.4f|%.4f|%.4f", solverDragGain, biasAt35m, biasAt45m, biasAt55m);
        }
    }

    private record CandidateResult(
            String stage,
            TuneParams params,
            ShotFocusMetrics.Metrics metrics) {
    }

    public static void main(String[] args) throws IOException {
        boolean reportOnly = args.length > 0 && "report".equalsIgnoreCase(args[0]);

        Constants.Shooter.hubPositionField = Constants.Field.HUB_CENTER_BLUE.getTranslation();

        ShotScenarioEvaluator evaluator = new ShotScenarioEvaluator();
        ShotScenarioEvaluator.EvaluationConfig baseConfig = ShotScenarioEvaluator.EvaluationConfig.fromConstants();

        List<ShotFocusMetrics.ScenarioPoint> tuneScenarios = ShotFocusMetrics.buildScenarioSet(
                TUNE_DISTANCES_M,
                BEARINGS_DEG,
                SPEEDS_MPS,
                TUNE_DIRECTION_POINTS,
                baseConfig.hubPositionField(),
                baseConfig.shooterOffsetRobot());
        List<ShotFocusMetrics.ScenarioPoint> verifyScenarios = ShotFocusMetrics.buildScenarioSet(
                VERIFY_DISTANCES_M,
                BEARINGS_DEG,
                SPEEDS_MPS,
                VERIFY_DIRECTION_POINTS,
                baseConfig.hubPositionField(),
                baseConfig.shooterOffsetRobot());

        TuneParams baselineParams = loadBaselineParams(baseConfig);
        Map<String, ShotFocusMetrics.Metrics> metricsCache = new HashMap<>();
        List<CandidateResult> evaluated = new ArrayList<>();

        ShotFocusMetrics.Metrics baselineTune = evaluateForParams(
                evaluator,
                baseConfig,
                tuneScenarios,
                baselineParams,
                metricsCache);
        ShotFocusMetrics.Metrics baselineVerify = evaluateForParams(
                evaluator,
                baseConfig,
                verifyScenarios,
                baselineParams,
                new HashMap<>());

        evaluated.add(new CandidateResult("baseline", baselineParams, baselineTune));
        printMetrics("Baseline (tune set)", baselineTune);
        printMetrics("Baseline (verify set)", baselineVerify);

        if (reportOnly) {
            writeCandidateCsv(evaluated, Path.of("build", "shot_plots", "focus_tune_candidates.csv"));
            writeSummaryFile(baselineParams, baselineTune, baselineVerify, baselineParams, baselineTune, baselineVerify);
            return;
        }

        TuneParams best = baselineParams;
        ShotFocusMetrics.Metrics bestMetrics = baselineTune;

        // Stage 1: coarse deterministic scan per parameter.
        best = optimizeParameter(
                "coarse_drag_gain",
                best,
                range(0.0, 1.2, 0.05),
                evaluator,
                baseConfig,
                tuneScenarios,
                metricsCache,
                evaluated);
        bestMetrics = evaluateForParams(evaluator, baseConfig, tuneScenarios, best, metricsCache);

        best = optimizeParameter(
                "coarse_bias_35",
                best,
                range(0.90, 1.30, 0.05),
                evaluator,
                baseConfig,
                tuneScenarios,
                metricsCache,
                evaluated);
        best = optimizeParameter(
                "coarse_bias_45",
                best,
                range(0.85, 1.45, 0.05),
                evaluator,
                baseConfig,
                tuneScenarios,
                metricsCache,
                evaluated);
        best = optimizeParameter(
                "coarse_bias_55",
                best,
                range(0.90, 2.10, 0.05),
                evaluator,
                baseConfig,
                tuneScenarios,
                metricsCache,
                evaluated);

        // Stage 2: fine local refinement around current best.
        best = optimizeParameter(
                "fine_drag_gain",
                best,
                rangeAround(best.solverDragGain(), 0.25, 0.02, 0.0, 1.8),
                evaluator,
                baseConfig,
                tuneScenarios,
                metricsCache,
                evaluated);
        best = optimizeParameter(
                "fine_bias_35",
                best,
                rangeAround(best.biasAt35m(), 0.16, 0.02, 0.75, 1.90),
                evaluator,
                baseConfig,
                tuneScenarios,
                metricsCache,
                evaluated);
        best = optimizeParameter(
                "fine_bias_45",
                best,
                rangeAround(best.biasAt45m(), 0.18, 0.02, 0.75, 2.00),
                evaluator,
                baseConfig,
                tuneScenarios,
                metricsCache,
                evaluated);
        best = optimizeParameter(
                "fine_bias_55",
                best,
                rangeAround(best.biasAt55m(), 0.30, 0.02, 0.75, 2.40),
                evaluator,
                baseConfig,
                tuneScenarios,
                metricsCache,
                evaluated);

        ShotFocusMetrics.Metrics bestTune = evaluateForParams(
                evaluator,
                baseConfig,
                tuneScenarios,
                best,
                metricsCache);
        ShotFocusMetrics.Metrics bestVerify = evaluateForParams(
                evaluator,
                baseConfig,
                verifyScenarios,
                best,
                new HashMap<>());

        printMetrics("Best (tune set)", bestTune);
        printMetrics("Best (verify set)", bestVerify);
        boolean gatePass = ShotFocusMetrics.passesFocusGate(
                bestVerify,
                FOCUS_GATE_HIT_RATE,
                FOCUS_GATE_MAX_INVALID_RATE,
                FOCUS_GATE_MAX_SATURATION_RATE);
        System.out.printf(
                Locale.US,
                "Focus gate pass (2-5m): %s [hit>=%.2f, invalid<=%.2f, saturation<=%.2f]%n",
                gatePass,
                FOCUS_GATE_HIT_RATE,
                FOCUS_GATE_MAX_INVALID_RATE,
                FOCUS_GATE_MAX_SATURATION_RATE);

        printRecommendation(best);
        writeCandidateCsv(evaluated, Path.of("build", "shot_plots", "focus_tune_candidates.csv"));
        writeSummaryFile(baselineParams, baselineTune, baselineVerify, best, bestTune, bestVerify);
    }

    private static TuneParams optimizeParameter(
            String stage,
            TuneParams start,
            double[] candidates,
            ShotScenarioEvaluator evaluator,
            ShotScenarioEvaluator.EvaluationConfig baseConfig,
            List<ShotFocusMetrics.ScenarioPoint> scenarios,
            Map<String, ShotFocusMetrics.Metrics> cache,
            List<CandidateResult> evaluated) {
        TuneParams best = start;
        ShotFocusMetrics.Metrics bestMetrics = evaluateForParams(evaluator, baseConfig, scenarios, best, cache);
        for (double candidate : candidates) {
            TuneParams params = switch (stage) {
                case "coarse_drag_gain", "fine_drag_gain" -> new TuneParams(
                        candidate,
                        start.biasAt35m(),
                        start.biasAt45m(),
                        start.biasAt55m());
                case "coarse_bias_35", "fine_bias_35" -> new TuneParams(
                        start.solverDragGain(),
                        candidate,
                        start.biasAt45m(),
                        start.biasAt55m());
                case "coarse_bias_45", "fine_bias_45" -> new TuneParams(
                        start.solverDragGain(),
                        start.biasAt35m(),
                        candidate,
                        start.biasAt55m());
                case "coarse_bias_55", "fine_bias_55" -> new TuneParams(
                        start.solverDragGain(),
                        start.biasAt35m(),
                        start.biasAt45m(),
                        candidate);
                default -> start;
            };
            ShotFocusMetrics.Metrics metrics = evaluateForParams(evaluator, baseConfig, scenarios, params, cache);
            evaluated.add(new CandidateResult(stage, params, metrics));
            if (isBetter(metrics, bestMetrics)) {
                best = params;
                bestMetrics = metrics;
            }
        }
        System.out.printf(
                Locale.US,
                "%s -> objective=%.2f, focusHit=%.3f, focusErr=%.3f, longHit=%.3f, longErr=%.3f, focusSat=%.3f, focusInvalid=%.3f%n",
                stage,
                bestMetrics.objective(),
                bestMetrics.focusHitRate(),
                bestMetrics.focusMeanClosestErrorMeters(),
                bestMetrics.longHitRate(),
                bestMetrics.longMeanClosestErrorMeters(),
                bestMetrics.focusSaturationRate(),
                bestMetrics.focusInvalidRate());
        return best;
    }

    private static boolean isBetter(ShotFocusMetrics.Metrics a, ShotFocusMetrics.Metrics b) {
        if (a.objective() > b.objective() + 1e-9) {
            return true;
        }
        if (Math.abs(a.objective() - b.objective()) <= 1e-9) {
            if (a.focusHitRate() > b.focusHitRate() + 1e-9) {
                return true;
            }
            if (Math.abs(a.focusHitRate() - b.focusHitRate()) <= 1e-9
                    && a.focusMeanClosestErrorMeters() < b.focusMeanClosestErrorMeters() - 1e-9) {
                return true;
            }
            if (Math.abs(a.focusHitRate() - b.focusHitRate()) <= 1e-9
                    && Math.abs(a.focusMeanClosestErrorMeters() - b.focusMeanClosestErrorMeters()) <= 1e-9
                    && a.longHitRate() > b.longHitRate() + 1e-9) {
                return true;
            }
        }
        return false;
    }

    private static ShotFocusMetrics.Metrics evaluateForParams(
            ShotScenarioEvaluator evaluator,
            ShotScenarioEvaluator.EvaluationConfig baseConfig,
            List<ShotFocusMetrics.ScenarioPoint> scenarios,
            TuneParams params,
            Map<String, ShotFocusMetrics.Metrics> cache) {
        String key = params.key() + "|" + scenarios.size();
        ShotFocusMetrics.Metrics cached = cache.get(key);
        if (cached != null) {
            return cached;
        }
        ShotScenarioEvaluator.EvaluationConfig tunedConfig = configForParams(baseConfig, params);
        ShotFocusMetrics.Metrics metrics = ShotFocusMetrics.evaluate(evaluator, tunedConfig, scenarios);
        cache.put(key, metrics);
        return metrics;
    }

    private static ShotScenarioEvaluator.EvaluationConfig configForParams(
            ShotScenarioEvaluator.EvaluationConfig baseConfig,
            TuneParams params) {
        Point2D[] biasPoints = new Point2D.Double[] {
                new Point2D.Double(1.5, Constants.Shooter.distanceScaleBiasTable.getOutput(1.5)),
                new Point2D.Double(2.5, Constants.Shooter.distanceScaleBiasTable.getOutput(2.5)),
                new Point2D.Double(3.5, params.biasAt35m()),
                new Point2D.Double(4.5, params.biasAt45m()),
                new Point2D.Double(5.5, params.biasAt55m())
        };
        return baseConfig
                .withDistanceScaleBiasPoints(biasPoints)
                .withSolverDragSpeedCompensationGain(params.solverDragGain());
    }

    private static TuneParams loadBaselineParams(ShotScenarioEvaluator.EvaluationConfig config) {
        return new TuneParams(
                config.solverDragSpeedCompensationGain(),
                config.distanceScaleBiasTable().getOutput(3.5),
                config.distanceScaleBiasTable().getOutput(4.5),
                config.distanceScaleBiasTable().getOutput(5.5));
    }

    private static double[] range(double start, double end, double step) {
        List<Double> vals = new ArrayList<>();
        for (double v = start; v <= end + 1e-9; v += step) {
            vals.add(v);
        }
        return vals.stream().mapToDouble(Double::doubleValue).toArray();
    }

    private static double[] rangeAround(double center, double radius, double step, double min, double max) {
        double lo = Math.max(min, center - radius);
        double hi = Math.min(max, center + radius);
        return range(lo, hi, step);
    }

    private static void printMetrics(String label, ShotFocusMetrics.Metrics metrics) {
        System.out.printf(
                Locale.US,
                "%s: objective=%.2f | hit=%.3f | err=%.3f | sat=%.3f | invalid=%.3f | focusHit=%.3f | focusErr=%.3f | focusSat=%.3f | focusInvalid=%.3f | longHit=%.3f | longErr=%.3f | longSat=%.3f | longInvalid=%.3f | p95=%.3f%n",
                label,
                metrics.objective(),
                metrics.hitRate(),
                metrics.meanClosestErrorMeters(),
                metrics.saturationRate(),
                metrics.invalidRate(),
                metrics.focusHitRate(),
                metrics.focusMeanClosestErrorMeters(),
                metrics.focusSaturationRate(),
                metrics.focusInvalidRate(),
                metrics.longHitRate(),
                metrics.longMeanClosestErrorMeters(),
                metrics.longSaturationRate(),
                metrics.longInvalidRate(),
                metrics.p95ClosestErrorMeters());
    }

    private static void printRecommendation(TuneParams params) {
        System.out.println();
        System.out.println("Recommended constants update:");
        System.out.printf(Locale.US, "solverDragSpeedCompensationGain = %.3f%n", params.solverDragGain());
        System.out.println("kDistanceScaleBiasPoints = {");
        System.out.printf(Locale.US, "  (1.5, %.3f)%n", Constants.Shooter.distanceScaleBiasTable.getOutput(1.5));
        System.out.printf(Locale.US, "  (2.5, %.3f)%n", Constants.Shooter.distanceScaleBiasTable.getOutput(2.5));
        System.out.printf(Locale.US, "  (3.5, %.3f)%n", params.biasAt35m());
        System.out.printf(Locale.US, "  (4.5, %.3f)%n", params.biasAt45m());
        System.out.printf(Locale.US, "  (5.5, %.3f)%n", params.biasAt55m());
        System.out.println("}");
    }

    private static void writeCandidateCsv(List<CandidateResult> rows, Path file) throws IOException {
        Files.createDirectories(file.getParent());
        List<String> lines = new ArrayList<>();
        lines.add(
                "stage,drag_gain,bias_3p5,bias_4p5,bias_5p5,objective,hit_rate,mean_error_m,saturation_rate,invalid_rate,focus_hit_rate,focus_mean_error_m,focus_saturation_rate,focus_invalid_rate,long_hit_rate,long_mean_error_m,long_saturation_rate,long_invalid_rate,p95_error_m");
        for (CandidateResult row : rows) {
            ShotFocusMetrics.Metrics m = row.metrics();
            lines.add(String.format(
                    Locale.US,
                    "%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                    row.stage(),
                    row.params().solverDragGain(),
                    row.params().biasAt35m(),
                    row.params().biasAt45m(),
                    row.params().biasAt55m(),
                    m.objective(),
                    m.hitRate(),
                    m.meanClosestErrorMeters(),
                    m.saturationRate(),
                    m.invalidRate(),
                    m.focusHitRate(),
                    m.focusMeanClosestErrorMeters(),
                    m.focusSaturationRate(),
                    m.focusInvalidRate(),
                    m.longHitRate(),
                    m.longMeanClosestErrorMeters(),
                    m.longSaturationRate(),
                    m.longInvalidRate(),
                    m.p95ClosestErrorMeters()));
        }
        Files.write(file, lines, StandardCharsets.UTF_8);
        System.out.println("Wrote candidate report: " + file.toAbsolutePath());
    }

    private static void writeSummaryFile(
            TuneParams baselineParams,
            ShotFocusMetrics.Metrics baselineTune,
            ShotFocusMetrics.Metrics baselineVerify,
            TuneParams bestParams,
            ShotFocusMetrics.Metrics bestTune,
            ShotFocusMetrics.Metrics bestVerify) throws IOException {
        Path file = Path.of("build", "shot_plots", "focus_tune_summary.txt");
        Files.createDirectories(file.getParent());
        List<String> lines = new ArrayList<>();
        lines.add("Shot Focus Auto Tune Summary");
        lines.add("============================");
        lines.add("");
        lines.add("Baseline parameters:");
        lines.add(String.format(Locale.US, "  dragGain=%.4f, bias3.5=%.4f, bias4.5=%.4f, bias5.5=%.4f",
                baselineParams.solverDragGain(),
                baselineParams.biasAt35m(),
                baselineParams.biasAt45m(),
                baselineParams.biasAt55m()));
        lines.add(String.format(Locale.US, "  tune objective=%.3f, verify objective=%.3f",
                baselineTune.objective(),
                baselineVerify.objective()));
        lines.add(String.format(Locale.US, "  verify focusHit=%.3f, focusErr=%.3f, focusSat=%.3f, focusInvalid=%.3f",
                baselineVerify.focusHitRate(),
                baselineVerify.focusMeanClosestErrorMeters(),
                baselineVerify.focusSaturationRate(),
                baselineVerify.focusInvalidRate()));
        lines.add(String.format(Locale.US, "  verify longHit=%.3f, longErr=%.3f, longSat=%.3f, longInvalid=%.3f",
                baselineVerify.longHitRate(),
                baselineVerify.longMeanClosestErrorMeters(),
                baselineVerify.longSaturationRate(),
                baselineVerify.longInvalidRate()));
        lines.add("");
        lines.add("Best parameters:");
        lines.add(String.format(Locale.US, "  dragGain=%.4f, bias3.5=%.4f, bias4.5=%.4f, bias5.5=%.4f",
                bestParams.solverDragGain(),
                bestParams.biasAt35m(),
                bestParams.biasAt45m(),
                bestParams.biasAt55m()));
        lines.add(String.format(Locale.US, "  tune objective=%.3f, verify objective=%.3f",
                bestTune.objective(),
                bestVerify.objective()));
        lines.add(String.format(Locale.US, "  verify focusHit=%.3f, focusErr=%.3f, focusSat=%.3f, focusInvalid=%.3f",
                bestVerify.focusHitRate(),
                bestVerify.focusMeanClosestErrorMeters(),
                bestVerify.focusSaturationRate(),
                bestVerify.focusInvalidRate()));
        lines.add(String.format(Locale.US, "  verify longHit=%.3f, longErr=%.3f, longSat=%.3f, longInvalid=%.3f",
                bestVerify.longHitRate(),
                bestVerify.longMeanClosestErrorMeters(),
                bestVerify.longSaturationRate(),
                bestVerify.longInvalidRate()));
        boolean gate = ShotFocusMetrics.passesFocusGate(
                bestVerify,
                FOCUS_GATE_HIT_RATE,
                FOCUS_GATE_MAX_INVALID_RATE,
                FOCUS_GATE_MAX_SATURATION_RATE);
        lines.add(String.format(Locale.US, "  focusGatePass=%s (hit>=%.2f, invalid<=%.2f, saturation<=%.2f)",
                gate,
                FOCUS_GATE_HIT_RATE,
                FOCUS_GATE_MAX_INVALID_RATE,
                FOCUS_GATE_MAX_SATURATION_RATE));
        Files.write(file, lines, StandardCharsets.UTF_8);
        System.out.println("Wrote summary: " + file.toAbsolutePath());
    }
}
