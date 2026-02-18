package frc.robot.sim;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;

/**
 * Runs one explicit shooter scenario and exports full trajectory/output metrics as JSON.
 *
 * <p>Args: xMeters,yMeters,headingDeg,vxMps,vyMps[,outputPathJson]
 */
public final class SingleScenarioRunner {
    private static final double SIM_DT_SEC = 0.005;
    private static final Path DEFAULT_OUTPUT = Path.of("build", "shot_plots", "single_scenario_output.json");

    private SingleScenarioRunner() {
    }

    public static void main(String[] args) throws IOException {
        double x = parseDoubleArg(args, 0, 2.0);
        double y = parseDoubleArg(args, 1, 3.0);
        double headingDeg = parseDoubleArg(args, 2, 0.0);
        double vx = parseDoubleArg(args, 3, 1.0);
        double vy = parseDoubleArg(args, 4, 0.0);
        Path outputPath = args.length >= 6 ? Path.of(args[5]) : DEFAULT_OUTPUT;

        Constants.Shooter.hubPositionField = Constants.Field.HUB_CENTER_BLUE.getTranslation();

        Pose2d pose = new Pose2d(x, y, Rotation2d.fromDegrees(headingDeg));
        Translation2d robotVelocity = new Translation2d(vx, vy);

        ShotScenarioEvaluator evaluator = new ShotScenarioEvaluator();
        ShotScenarioEvaluator.EvaluationConfig config = ShotScenarioEvaluator.EvaluationConfig.fromConstants();
        ShotScenarioEvaluator.Scenario scenario = new ShotScenarioEvaluator.Scenario("single", pose, robotVelocity);
        ShotScenarioEvaluator.Evaluation evaluation = evaluator.evaluate(scenario, config);

        List<ShotSample> trace = new ArrayList<>();
        Optional<ShotResult> shotResult = Optional.empty();
        if (evaluation.solveStatus() == frc.robot.utils.ShotKinematicSolver.SolveStatus.SOLVED) {
            ShotTraceComputation traceData = simulateTrace(scenario, config, evaluation);
            trace = traceData.trace();
            shotResult = traceData.result();
        }

        String json = buildJson(
                x, y, headingDeg, vx, vy,
                evaluation,
                shotResult,
                trace,
                config.hubPositionField(),
                config.hubRadiusMeters());

        if (outputPath.getParent() != null) {
            Files.createDirectories(outputPath.getParent());
        }
        Files.writeString(outputPath, json, StandardCharsets.UTF_8);

        System.out.printf(
                Locale.US,
                "single scenario exported: %s | hit=%s | solve=%s | top=%.2f | bottom=%.2f%n",
                outputPath.toAbsolutePath(),
                shotResult.map(ShotResult::hit).orElse(false),
                evaluation.solveStatus(),
                evaluation.topRpm(),
                evaluation.bottomRpm());
    }

    private static ShotTraceComputation simulateTrace(
            ShotScenarioEvaluator.Scenario scenario,
            ShotScenarioEvaluator.EvaluationConfig config,
            ShotScenarioEvaluator.Evaluation evaluation) {
        Translation2d shooterField = scenario.robotPoseField().getTranslation()
                .plus(config.shooterOffsetRobot().rotateBy(scenario.robotPoseField().getRotation()));

        double yawFieldRad = scenario.robotPoseField().getRotation().getRadians()
                + Math.toRadians(evaluation.turretYawRobotDeg());
        double muzzleSpeedMps = Math.max(0.0, evaluation.muzzleSpeedMps());
        double horizontalSpeedMps = muzzleSpeedMps * Math.cos(config.launchPitchRad());
        double verticalSpeedMps = muzzleSpeedMps * Math.sin(config.launchPitchRad());
        Translation2d muzzleVelocityField = new Translation2d(horizontalSpeedMps, new Rotation2d(yawFieldRad));
        Translation2d initialVelocityField = scenario.robotVelocityFieldMps().plus(muzzleVelocityField);

        BallisticShotSimulator simulator = new BallisticShotSimulator(
                config.hubPositionField(),
                config.hubCenterHeightMeters(),
                config.hubRadiusMeters(),
                config.hubHeightToleranceMeters(),
                config.gravityMetersPerSec2(),
                config.maxFlightTimeSec(),
                config.ballisticDragCoeffPerMeter());

        simulator.queueShot(new ShotEvent(
                0.0,
                new Translation3d(shooterField.getX(), shooterField.getY(), config.launchHeightMeters()),
                new Translation3d(initialVelocityField.getX(), initialVelocityField.getY(), verticalSpeedMps),
                Math.toRadians(evaluation.turretYawRobotDeg()),
                evaluation.topRpm(),
                evaluation.bottomRpm()));

        Optional<CompletedShotTrace> completed = Optional.empty();
        double elapsed = 0.0;
        while (elapsed <= config.maxFlightTimeSec() + SIM_DT_SEC) {
            simulator.update(SIM_DT_SEC);
            List<CompletedShotTrace> traces = simulator.drainCompletedShotTraces();
            if (!traces.isEmpty()) {
                completed = Optional.of(traces.get(0));
                break;
            }
            elapsed += SIM_DT_SEC;
        }

        if (completed.isEmpty()) {
            return new ShotTraceComputation(List.of(), Optional.empty());
        }
        return new ShotTraceComputation(
                completed.get().samples(),
                Optional.of(completed.get().result()));
    }

    private static String buildJson(
            double x,
            double y,
            double headingDeg,
            double vx,
            double vy,
            ShotScenarioEvaluator.Evaluation eval,
            Optional<ShotResult> shotResult,
            List<ShotSample> trace,
            Translation2d hubCenter,
            double hubRadiusMeters) {
        StringBuilder sb = new StringBuilder(16_384);
        sb.append("{\n");
        sb.append("  \"input\": {\n");
        sb.append(String.format(Locale.US, "    \"x_m\": %.6f,%n", x));
        sb.append(String.format(Locale.US, "    \"y_m\": %.6f,%n", y));
        sb.append(String.format(Locale.US, "    \"heading_deg\": %.6f,%n", headingDeg));
        sb.append(String.format(Locale.US, "    \"vx_mps\": %.6f,%n", vx));
        sb.append(String.format(Locale.US, "    \"vy_mps\": %.6f%n", vy));
        sb.append("  },\n");

        sb.append("  \"hub\": {\n");
        sb.append(String.format(Locale.US, "    \"x_m\": %.6f,%n", hubCenter.getX()));
        sb.append(String.format(Locale.US, "    \"y_m\": %.6f,%n", hubCenter.getY()));
        sb.append(String.format(Locale.US, "    \"radius_m\": %.6f%n", hubRadiusMeters));
        sb.append("  },\n");

        sb.append("  \"command\": {\n");
        sb.append(String.format(Locale.US, "    \"solve_status\": \"%s\",%n", eval.solveStatus().name()));
        sb.append(String.format(Locale.US, "    \"turret_yaw_deg\": %.6f,%n", eval.turretYawRobotDeg()));
        sb.append(String.format(Locale.US, "    \"top_rpm\": %.6f,%n", eval.topRpm()));
        sb.append(String.format(Locale.US, "    \"bottom_rpm\": %.6f,%n", eval.bottomRpm()));
        sb.append(String.format(Locale.US, "    \"required_muzzle_speed_mps\": %.6f,%n", eval.requiredMuzzleSpeedMps()));
        sb.append(String.format(Locale.US, "    \"actual_muzzle_speed_mps\": %.6f,%n", eval.muzzleSpeedMps()));
        sb.append(String.format(Locale.US, "    \"rpm_saturated_top\": %s,%n", eval.rpmSaturatedTop()));
        sb.append(String.format(Locale.US, "    \"rpm_saturated_bottom\": %s,%n", eval.rpmSaturatedBottom()));
        sb.append(String.format(Locale.US, "    \"rpm_saturated_any\": %s%n", eval.rpmSaturatedAny()));
        sb.append("  },\n");

        sb.append("  \"result\": {\n");
        sb.append(String.format(Locale.US, "    \"hit\": %s,%n", shotResult.map(ShotResult::hit).orElse(false)));
        sb.append(String.format(Locale.US, "    \"closest_distance_m\": %.6f,%n",
                shotResult.map(ShotResult::closestDistanceMeters).orElse(Double.POSITIVE_INFINITY)));
        sb.append(String.format(Locale.US, "    \"horizontal_error_m\": %.6f,%n",
                shotResult.map(ShotResult::horizontalErrorMeters).orElse(Double.POSITIVE_INFINITY)));
        sb.append(String.format(Locale.US, "    \"vertical_error_m\": %.6f,%n",
                shotResult.map(ShotResult::verticalErrorMeters).orElse(Double.POSITIVE_INFINITY)));
        sb.append(String.format(Locale.US, "    \"flight_time_s\": %.6f,%n",
                shotResult.map(ShotResult::flightTimeSec).orElse(0.0)));
        sb.append(String.format(Locale.US, "    \"time_of_closest_approach_s\": %.6f%n",
                shotResult.map(ShotResult::timeOfClosestApproachSec).orElse(0.0)));
        sb.append("  },\n");

        sb.append("  \"trajectory\": [\n");
        for (int i = 0; i < trace.size(); i++) {
            ShotSample sample = trace.get(i);
            sb.append(String.format(
                    Locale.US,
                    "    {\"t_s\": %.6f, \"x_m\": %.6f, \"y_m\": %.6f, \"z_m\": %.6f}%s%n",
                    sample.timeSec(),
                    sample.positionField().getX(),
                    sample.positionField().getY(),
                    sample.positionField().getZ(),
                    (i + 1 == trace.size()) ? "" : ","));
        }
        sb.append("  ]\n");
        sb.append("}\n");
        return sb.toString();
    }

    private static double parseDoubleArg(String[] args, int idx, double fallback) {
        if (args.length <= idx) {
            return fallback;
        }
        try {
            return Double.parseDouble(args[idx].trim());
        } catch (NumberFormatException ignored) {
            return fallback;
        }
    }

    private record ShotTraceComputation(List<ShotSample> trace, Optional<ShotResult> result) {
    }
}

