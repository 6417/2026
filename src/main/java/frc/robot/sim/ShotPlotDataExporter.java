package frc.robot.sim;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;

/**
 * Exports shot simulation data (RPM sweep + trajectories) to CSV files for plotting.
 *
 * <p>Generated datasets are used for:
 * - curve-shape inspection (rpm_distance / trajectories),
 * - direction-dependent behavior (direction_sweep),
 * - robustness statistics across distance/speed/bearing (robustness_grid/summary).
 */
public class ShotPlotDataExporter {
    private static final int NUM_POINTS = 50;
    private static final double RPM_MIN = 2500.0;
    private static final double RPM_MAX = 6000.0;
    private static final double DT_SEC = 0.005;
    private static final int DIRECTION_POINTS = 73;
    private static final double[] SWEEP_SPEEDS_MPS = { 0.0, 1.0, 2.0, 3.0 };
    private static final int ROBUST_DIRECTION_POINTS = 37;
    private static final double[] ROBUST_SPEEDS_MPS = { 0.0, 1.0, 2.0, 3.0 };
    private static final double[] ROBUST_DISTANCES_M = { 2.0, 3.0, 4.0, 5.0, 6.0 };
    private static final double[] ROBUST_BEARINGS_DEG = { -180.0, -135.0, -90.0, -45.0, 0.0, 45.0, 90.0, 135.0 };
    private static final double SWEEP_SHOOTER_DISTANCE_METERS = 4.0;
    private static final Rotation2d SWEEP_BEARING_FROM_HUB = Rotation2d.fromDegrees(0.0);
    private static final Pose2d SWEEP_POSE = buildSweepPose();

    private record Sample(double tSec, double xMeters, double zMeters) {
    }

    private record TrajectoryMetrics(
            double distanceAtHubHeightMeters,
            double rangeAtGroundMeters,
            double maxHeightMeters,
            double timeAtHubHeightSec) {
    }

    private record ScaleInfo(double rawScale, double appliedScale, boolean clamped) {
    }

    private static final class SummaryCell {
        int count = 0;
        int hitCount = 0;
        int clampedCount = 0;
        double sumClosestError = 0.0;

        void add(boolean hit, boolean clamped, double closestErrorMeters) {
            count++;
            if (hit) {
                hitCount++;
            }
            if (clamped) {
                clampedCount++;
            }
            sumClosestError += closestErrorMeters;
        }
    }

    public static void main(String[] args) throws IOException {
        Path outputDir = Path.of("build", "shot_plots");
        Files.createDirectories(outputDir);

        Path rpmDistanceCsv = outputDir.resolve("rpm_distance.csv");
        Path trajectoriesCsv = outputDir.resolve("trajectories.csv");
        Path directionSweepCsv = outputDir.resolve("direction_sweep.csv");
        Path robustnessGridCsv = outputDir.resolve("robustness_grid.csv");
        Path robustnessSummaryCsv = outputDir.resolve("robustness_summary.csv");

        List<String> distanceRows = new ArrayList<>();
        List<String> trajectoryRows = new ArrayList<>();
        List<String> directionRows = new ArrayList<>();
        List<String> robustnessRows = new ArrayList<>();
        List<String> robustnessSummaryRows = new ArrayList<>();
        Map<String, SummaryCell> summaryByDistanceSpeed = new HashMap<>();
        distanceRows.add("rpm,distance_at_hub_height_m,range_at_ground_m,max_height_m,time_at_hub_height_s,hub_height_m");
        trajectoryRows.add("rpm,time_s,x_m,z_m");
        directionRows.add(
                "speed_mps,direction_deg,vx_mps,vy_mps,distance_m,raw_scale,applied_scale,scale_clamped,turret_yaw_deg,top_rpm,bottom_rpm,muzzle_speed_mps,closest_error_m,horizontal_error_m,vertical_error_m,flight_time_s,hit");
        robustnessRows.add(
                "distance_m,bearing_deg,speed_mps,direction_deg,vx_mps,vy_mps,raw_scale,applied_scale,scale_clamped,turret_yaw_deg,top_rpm,bottom_rpm,muzzle_speed_mps,closest_error_m,horizontal_error_m,vertical_error_m,flight_time_s,hit");
        robustnessSummaryRows.add("distance_m,speed_mps,samples,hit_rate_pct,mean_closest_error_m,clamp_rate_pct");

        // 1) RPM sweep dataset for scalar trajectory behavior.
        for (int i = 0; i < NUM_POINTS; i++) {
            double rpm = RPM_MIN + ((RPM_MAX - RPM_MIN) * i / (NUM_POINTS - 1));
            double muzzleSpeedMps = rpm * Constants.Shooter.rpmToMpsFactor;
            List<Sample> samples = simulateTrajectory(muzzleSpeedMps);
            TrajectoryMetrics metrics = computeMetrics(samples);

            distanceRows.add(String.format(
                    Locale.US,
                    "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f",
                    rpm,
                    metrics.distanceAtHubHeightMeters(),
                    metrics.rangeAtGroundMeters(),
                    metrics.maxHeightMeters(),
                    metrics.timeAtHubHeightSec(),
                    Constants.ShooterSim.hubCenterHeightMeters));

            for (Sample sample : samples) {
                trajectoryRows.add(String.format(
                        Locale.US,
                        "%.6f,%.6f,%.6f,%.6f",
                        rpm,
                        sample.tSec(),
                        sample.xMeters(),
                        sample.zMeters()));
            }
        }

        ShotScenarioEvaluator evaluator = new ShotScenarioEvaluator();
        // 2) Single-distance direction sweep: good for visualizing curve smoothness.
        for (double speedMps : SWEEP_SPEEDS_MPS) {
            for (int i = 0; i < DIRECTION_POINTS; i++) {
                double directionDeg = -180.0 + (360.0 * i / (DIRECTION_POINTS - 1));
                Translation2d velocity = new Translation2d(speedMps, Rotation2d.fromDegrees(directionDeg));
                ShotScenarioEvaluator.Scenario scenario = new ShotScenarioEvaluator.Scenario(
                        String.format(Locale.US, "s%.1f_d%.1f", speedMps, directionDeg),
                        SWEEP_POSE,
                        velocity);
                ShotScenarioEvaluator.Evaluation eval = evaluator.evaluate(scenario);

                Translation2d shooterPos = scenario.robotPoseField().getTranslation().plus(
                        Constants.Shooter.shooterOffsetRobot.rotateBy(scenario.robotPoseField().getRotation()));
                Translation2d toHub = Constants.Shooter.hubPositionField.minus(shooterPos);
                double distance = toHub.getNorm();
                double flightTime = Math.max(
                        Constants.Shooter.minFlightTimeSec,
                        Constants.Shooter.flightTimeTable.getOutput(distance));
                Translation2d requiredBallVelocity = toHub.div(flightTime);
                Translation2d requiredMuzzleVelocity = requiredBallVelocity.minus(velocity);
                double requiredMuzzleSpeed = requiredMuzzleVelocity.getNorm();
                ScaleInfo scaleInfo = computeScale(distance, requiredMuzzleSpeed);

                directionRows.add(String.format(
                        Locale.US,
                        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s",
                        speedMps,
                        directionDeg,
                        velocity.getX(),
                        velocity.getY(),
                        distance,
                        scaleInfo.rawScale(),
                        scaleInfo.appliedScale(),
                        scaleInfo.clamped(),
                        eval.turretYawRobotDeg(),
                        eval.topRpm(),
                        eval.bottomRpm(),
                        eval.muzzleSpeedMps(),
                        eval.closestDistanceMeters(),
                        eval.horizontalErrorMeters(),
                        eval.verticalErrorMeters(),
                        eval.flightTimeSec(),
                        eval.hit()));
            }
        }

        // 3) Robustness grid: broad sample across distance, bearing, speed, and direction.
        for (double distanceM : ROBUST_DISTANCES_M) {
            for (double bearingDeg : ROBUST_BEARINGS_DEG) {
                Pose2d pose = buildPoseFromHub(distanceM, Rotation2d.fromDegrees(bearingDeg), Rotation2d.fromDegrees(0.0));
                for (double speedMps : ROBUST_SPEEDS_MPS) {
                    for (int i = 0; i < ROBUST_DIRECTION_POINTS; i++) {
                        double directionDeg = -180.0 + (360.0 * i / (ROBUST_DIRECTION_POINTS - 1));
                        Translation2d velocity = new Translation2d(speedMps, Rotation2d.fromDegrees(directionDeg));
                        ShotScenarioEvaluator.Scenario scenario = new ShotScenarioEvaluator.Scenario(
                                String.format(Locale.US, "r_d%.1f_b%.1f_s%.1f_dir%.1f", distanceM, bearingDeg, speedMps, directionDeg),
                                pose,
                                velocity);
                        ShotScenarioEvaluator.Evaluation eval = evaluator.evaluate(scenario);

                        Translation2d shooterPos = scenario.robotPoseField().getTranslation().plus(
                                Constants.Shooter.shooterOffsetRobot.rotateBy(scenario.robotPoseField().getRotation()));
                        Translation2d toHub = Constants.Shooter.hubPositionField.minus(shooterPos);
                        double distance = toHub.getNorm();
                        double flightTime = Math.max(
                                Constants.Shooter.minFlightTimeSec,
                                Constants.Shooter.flightTimeTable.getOutput(distance));
                        Translation2d requiredBallVelocity = toHub.div(flightTime);
                        Translation2d requiredMuzzleVelocity = requiredBallVelocity.minus(velocity);
                        double requiredMuzzleSpeed = requiredMuzzleVelocity.getNorm();
                        ScaleInfo scaleInfo = computeScale(distance, requiredMuzzleSpeed);

                        robustnessRows.add(String.format(
                                Locale.US,
                                "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s",
                                distanceM,
                                bearingDeg,
                                speedMps,
                                directionDeg,
                                velocity.getX(),
                                velocity.getY(),
                                scaleInfo.rawScale(),
                                scaleInfo.appliedScale(),
                                scaleInfo.clamped(),
                                eval.turretYawRobotDeg(),
                                eval.topRpm(),
                                eval.bottomRpm(),
                                eval.muzzleSpeedMps(),
                                eval.closestDistanceMeters(),
                                eval.horizontalErrorMeters(),
                                eval.verticalErrorMeters(),
                                eval.flightTimeSec(),
                                eval.hit()));

                        String summaryKey = String.format(Locale.US, "%.3f|%.3f", distanceM, speedMps);
                        SummaryCell cell = summaryByDistanceSpeed.computeIfAbsent(summaryKey, key -> new SummaryCell());
                        cell.add(eval.hit(), scaleInfo.clamped(), eval.closestDistanceMeters());
                    }
                }
            }
        }

        // 4) Aggregate robustness summary per (distance,speed) cell.
        for (double distanceM : ROBUST_DISTANCES_M) {
            for (double speedMps : ROBUST_SPEEDS_MPS) {
                String summaryKey = String.format(Locale.US, "%.3f|%.3f", distanceM, speedMps);
                SummaryCell cell = summaryByDistanceSpeed.get(summaryKey);
                if (cell == null || cell.count == 0) {
                    continue;
                }
                double hitRatePct = (100.0 * cell.hitCount) / cell.count;
                double clampRatePct = (100.0 * cell.clampedCount) / cell.count;
                double meanErr = cell.sumClosestError / cell.count;
                robustnessSummaryRows.add(String.format(
                        Locale.US,
                        "%.6f,%.6f,%d,%.6f,%.6f,%.6f",
                        distanceM,
                        speedMps,
                        cell.count,
                        hitRatePct,
                        meanErr,
                        clampRatePct));
            }
        }

        Files.write(rpmDistanceCsv, distanceRows, StandardCharsets.UTF_8);
        Files.write(trajectoriesCsv, trajectoryRows, StandardCharsets.UTF_8);
        Files.write(directionSweepCsv, directionRows, StandardCharsets.UTF_8);
        Files.write(robustnessGridCsv, robustnessRows, StandardCharsets.UTF_8);
        Files.write(robustnessSummaryCsv, robustnessSummaryRows, StandardCharsets.UTF_8);

        System.out.println("Exported CSV files:");
        System.out.println(" - " + rpmDistanceCsv.toAbsolutePath());
        System.out.println(" - " + trajectoriesCsv.toAbsolutePath());
        System.out.println(" - " + directionSweepCsv.toAbsolutePath());
        System.out.println(" - " + robustnessGridCsv.toAbsolutePath());
        System.out.println(" - " + robustnessSummaryCsv.toAbsolutePath());
    }

    private static Pose2d buildSweepPose() {
        return buildPoseFromHub(SWEEP_SHOOTER_DISTANCE_METERS, SWEEP_BEARING_FROM_HUB, Rotation2d.fromDegrees(0.0));
    }

    private static Pose2d buildPoseFromHub(double shooterDistanceMeters, Rotation2d bearingFromHub, Rotation2d robotHeading) {
        Translation2d shooterPos = Constants.Shooter.hubPositionField.minus(
                new Translation2d(shooterDistanceMeters, bearingFromHub));
        Translation2d robotPos = shooterPos.minus(Constants.Shooter.shooterOffsetRobot.rotateBy(robotHeading));
        return new Pose2d(robotPos, robotHeading);
    }

    private static ScaleInfo computeScale(double distanceMeters, double requiredMuzzleSpeedMps) {
        double baseTopRpm = Constants.Shooter.topRpmTable.getOutput(distanceMeters);
        double baseBottomRpm = Constants.Shooter.bottomRpmTable.getOutput(distanceMeters);
        double nominalMuzzleSpeed = ((baseTopRpm + baseBottomRpm) * 0.5) * Constants.Shooter.rpmToMpsFactor;

        double rawScale = 1.0;
        if (nominalMuzzleSpeed > 1e-6) {
            rawScale = requiredMuzzleSpeedMps / nominalMuzzleSpeed;
        }
        // Distance-bias term matches runtime moving-shot math.
        rawScale *= Constants.Shooter.getDistanceScaleBias(distanceMeters);
        double appliedScale = MathUtil.clamp(
                rawScale,
                Constants.Shooter.movingShotScaleMin,
                Constants.Shooter.movingShotScaleMax);
        boolean scaleClamped = Math.abs(appliedScale - rawScale) > 1e-6;
        return new ScaleInfo(rawScale, appliedScale, scaleClamped);
    }

    private static List<Sample> simulateTrajectory(double muzzleSpeedMps) {
        List<Sample> samples = new ArrayList<>();
        double pitchRad = Constants.ShooterSim.launchPitchRad;
        double g = Constants.ShooterSim.gravityMetersPerSec2;
        double drag = Constants.ShooterSim.dragCoefficientPerMeter;

        double t = 0.0;
        double x = 0.0;
        double z = Constants.ShooterSim.launchHeightMeters;
        double vx = muzzleSpeedMps * Math.cos(pitchRad);
        double vz = muzzleSpeedMps * Math.sin(pitchRad);

        samples.add(new Sample(t, x, z));
        while (t < Constants.ShooterSim.maxFlightTimeSec && z >= 0.0) {
            double speed = Math.hypot(vx, vz);
            double dragFactor = drag * speed;
            double ax = -dragFactor * vx;
            double az = -dragFactor * vz - g;

            vx += ax * DT_SEC;
            vz += az * DT_SEC;
            x += vx * DT_SEC;
            z += vz * DT_SEC;
            t += DT_SEC;

            samples.add(new Sample(t, x, z));
        }
        return samples;
    }

    private static TrajectoryMetrics computeMetrics(List<Sample> samples) {
        double hubHeight = Constants.ShooterSim.hubCenterHeightMeters;
        double maxHeight = 0.0;
        int peakIndex = 0;
        for (int i = 0; i < samples.size(); i++) {
            if (samples.get(i).zMeters() > maxHeight) {
                maxHeight = samples.get(i).zMeters();
                peakIndex = i;
            }
        }

        double distanceAtHubHeight = 0.0;
        double timeAtHubHeight = 0.0;
        if (maxHeight >= hubHeight) {
            double minError = Double.POSITIVE_INFINITY;
            for (int i = peakIndex; i < samples.size(); i++) {
                double err = Math.abs(samples.get(i).zMeters() - hubHeight);
                if (err < minError) {
                    minError = err;
                    distanceAtHubHeight = samples.get(i).xMeters();
                    timeAtHubHeight = samples.get(i).tSec();
                }
            }
        }

        double rangeAtGround = samples.get(samples.size() - 1).xMeters();
        for (int i = 1; i < samples.size(); i++) {
            Sample prev = samples.get(i - 1);
            Sample cur = samples.get(i);
            if (prev.zMeters() >= 0.0 && cur.zMeters() < 0.0) {
                double alpha = prev.zMeters() / (prev.zMeters() - cur.zMeters());
                rangeAtGround = prev.xMeters() + alpha * (cur.xMeters() - prev.xMeters());
                break;
            }
        }

        return new TrajectoryMetrics(distanceAtHubHeight, rangeAtGround, maxHeight, timeAtHubHeight);
    }
}
