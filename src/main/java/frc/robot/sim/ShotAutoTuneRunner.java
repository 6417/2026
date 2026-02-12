package frc.robot.sim;

import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.utils.LinearInterpolationTable;

/**
 * Offline auto-tuner for shooter interpolation tables and simulator coefficients.
 *
 * <p>Runs a deterministic random search over:
 * - average wheel RPM table points,
 * - flight time table points,
 * - rpmToMpsFactor,
 * - dragCoefficientPerMeter.
 *
 * <p>It prints recommended constants that can be copied into Constants.java.
 */
public class ShotAutoTuneRunner {
    private static final double[] DISTANCE_POINTS_M = {1.5, 2.5, 3.5, 4.5, 5.5};
    private static final double[] TOP_BOTTOM_SPLIT_RPM = {200.0, 200.0, 300.0, 350.0, 400.0};
    private static final double EVAL_DT_SEC = 0.005;

    private static final int ITERATIONS = 5000;
    private static final long RNG_SEED = 20260212L;

    private static final class Params {
        final double[] avgRpm = new double[DISTANCE_POINTS_M.length];
        final double[] flightTimeSec = new double[DISTANCE_POINTS_M.length];
        double rpmToMpsFactor;
        double dragCoeffPerMeter;

        Params copy() {
            Params c = new Params();
            System.arraycopy(avgRpm, 0, c.avgRpm, 0, avgRpm.length);
            System.arraycopy(flightTimeSec, 0, c.flightTimeSec, 0, flightTimeSec.length);
            c.rpmToMpsFactor = rpmToMpsFactor;
            c.dragCoeffPerMeter = dragCoeffPerMeter;
            return c;
        }
    }

    private static final class Score {
        final double objective;
        final double hitRate;
        final double meanClosest;
        final double meanHorizontal;
        final double meanVertical;

        Score(double objective, double hitRate, double meanClosest, double meanHorizontal, double meanVertical) {
            this.objective = objective;
            this.hitRate = hitRate;
            this.meanClosest = meanClosest;
            this.meanHorizontal = meanHorizontal;
            this.meanVertical = meanVertical;
        }
    }

    public static void main(String[] args) {
        // Tune against blue hub; geometry is mirrored for red.
        Constants.Shooter.hubPositionField = Constants.Field.HUB_CENTER_BLUE.getTranslation();

        List<ShotScenarioEvaluator.Scenario> scenarios = buildScenarioSet();
        Params current = initialParams();
        Score currentScore = evaluate(current, scenarios);
        Params best = current.copy();
        Score bestScore = currentScore;

        Random rng = new Random(RNG_SEED);
        double avgStep = 220.0;
        double flightStep = 0.050;
        double rpmStep = 0.00022;
        double dragStep = 0.0060;

        for (int iter = 0; iter < ITERATIONS; iter++) {
            Params candidate = current.copy();
            mutate(candidate, rng, avgStep, flightStep, rpmStep, dragStep);
            projectAndClamp(candidate);

            Score candidateScore = evaluate(candidate, scenarios);
            double temp = 1.0 - ((double) iter / (double) ITERATIONS);
            temp = Math.max(0.02, temp);
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

            if ((iter + 1) % 500 == 0) {
                avgStep *= 0.87;
                flightStep *= 0.90;
                rpmStep *= 0.88;
                dragStep *= 0.88;
                System.out.printf(
                        "iter=%d | best hitRate=%.3f | meanClosest=%.3f | meanV=%.3f | rpmToMps=%.6f | drag=%.4f%n",
                        iter + 1,
                        bestScore.hitRate,
                        bestScore.meanClosest,
                        bestScore.meanVertical,
                        best.rpmToMpsFactor,
                        best.dragCoeffPerMeter);
            }
        }

        System.out.println();
        printRecommendation(best, bestScore, scenarios.size());
    }

    private static Params initialParams() {
        Params p = new Params();
        // Start from current table averages.
        p.avgRpm[0] = (2800.0 + 2600.0) * 0.5;
        p.avgRpm[1] = (3200.0 + 3000.0) * 0.5;
        p.avgRpm[2] = (3700.0 + 3400.0) * 0.5;
        p.avgRpm[3] = (4200.0 + 3850.0) * 0.5;
        p.avgRpm[4] = (4600.0 + 4200.0) * 0.5;

        p.flightTimeSec[0] = 0.30;
        p.flightTimeSec[1] = 0.36;
        p.flightTimeSec[2] = 0.43;
        p.flightTimeSec[3] = 0.50;
        p.flightTimeSec[4] = 0.57;

        p.rpmToMpsFactor = Constants.Shooter.rpmToMpsFactor;
        p.dragCoeffPerMeter = Constants.ShooterSim.dragCoefficientPerMeter;
        projectAndClamp(p);
        return p;
    }

    private static void mutate(
            Params p,
            Random rng,
            double avgStep,
            double flightStep,
            double rpmStep,
            double dragStep) {
        int variableCount = (2 * DISTANCE_POINTS_M.length) + 2;
        int which = rng.nextInt(variableCount);
        double sign = rng.nextBoolean() ? 1.0 : -1.0;
        if (which < DISTANCE_POINTS_M.length) {
            p.avgRpm[which] += sign * avgStep * (0.35 + 0.65 * rng.nextDouble());
        } else if (which < 2 * DISTANCE_POINTS_M.length) {
            int i = which - DISTANCE_POINTS_M.length;
            p.flightTimeSec[i] += sign * flightStep * (0.35 + 0.65 * rng.nextDouble());
        } else if (which == 2 * DISTANCE_POINTS_M.length) {
            p.rpmToMpsFactor += sign * rpmStep * (0.35 + 0.65 * rng.nextDouble());
        } else {
            p.dragCoeffPerMeter += sign * dragStep * (0.35 + 0.65 * rng.nextDouble());
        }
    }

    private static void projectAndClamp(Params p) {
        for (int i = 0; i < p.avgRpm.length; i++) {
            p.avgRpm[i] = MathUtil.clamp(p.avgRpm[i], 1800.0, 6500.0);
            p.flightTimeSec[i] = MathUtil.clamp(p.flightTimeSec[i], 0.18, 1.10);
        }
        for (int i = 1; i < p.avgRpm.length; i++) {
            p.avgRpm[i] = Math.max(p.avgRpm[i], p.avgRpm[i - 1] + 40.0);
            p.flightTimeSec[i] = Math.max(p.flightTimeSec[i], p.flightTimeSec[i - 1] + 0.015);
        }
        p.rpmToMpsFactor = MathUtil.clamp(p.rpmToMpsFactor, 0.0020, 0.0065);
        p.dragCoeffPerMeter = MathUtil.clamp(p.dragCoeffPerMeter, 0.000, 0.10);
    }

    private static Score evaluate(Params p, List<ShotScenarioEvaluator.Scenario> scenarios) {
        Point2D[] avgPoints = toPoints(DISTANCE_POINTS_M, p.avgRpm);
        Point2D[] flightPoints = toPoints(DISTANCE_POINTS_M, p.flightTimeSec);
        LinearInterpolationTable avgRpmTable = new LinearInterpolationTable(avgPoints);
        LinearInterpolationTable flightTimeTable = new LinearInterpolationTable(flightPoints);

        int hits = 0;
        double sumClosest = 0.0;
        double sumHorizontal = 0.0;
        double sumVertical = 0.0;

        for (ShotScenarioEvaluator.Scenario scenario : scenarios) {
            ShotResult result = evaluateScenario(scenario, avgRpmTable, flightTimeTable, p);
            if (result.hit()) {
                hits++;
            }
            sumClosest += result.closestDistanceMeters();
            sumHorizontal += result.horizontalErrorMeters();
            sumVertical += result.verticalErrorMeters();
        }

        int n = scenarios.size();
        double hitRate = (double) hits / (double) n;
        double meanClosest = sumClosest / n;
        double meanHorizontal = sumHorizontal / n;
        double meanVertical = sumVertical / n;

        // Primary objective is hit-rate; errors refine tie-breakers.
        double objective = (hitRate * 1000.0)
                - (meanClosest * 110.0)
                - (meanHorizontal * 50.0)
                - (meanVertical * 75.0);

        return new Score(objective, hitRate, meanClosest, meanHorizontal, meanVertical);
    }

    private static ShotResult evaluateScenario(
            ShotScenarioEvaluator.Scenario scenario,
            LinearInterpolationTable avgRpmTable,
            LinearInterpolationTable flightTimeTable,
            Params p) {
        Translation2d shooterPositionField = scenario.robotPoseField().getTranslation().plus(
                Constants.Shooter.shooterOffsetRobot.rotateBy(scenario.robotPoseField().getRotation()));
        Translation2d shooterToHubField = Constants.Shooter.hubPositionField.minus(shooterPositionField);
        double distanceMeters = shooterToHubField.getNorm();

        if (distanceMeters < 1e-6) {
            return new ShotResult(false, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, 0.0,
                    0.0);
        }

        double baseAvgRpm = avgRpmTable.getOutput(distanceMeters);
        double flightTimeEstimateSec = Math.max(Constants.Shooter.minFlightTimeSec, flightTimeTable.getOutput(distanceMeters));

        Translation2d requiredBallVelocityField = shooterToHubField.div(flightTimeEstimateSec);
        Translation2d requiredMuzzleVelocityField = requiredBallVelocityField.minus(scenario.robotVelocityFieldMps());

        double yawFieldRad = requiredMuzzleVelocityField.getAngle().getRadians();
        double requiredMuzzleSpeedMps = requiredMuzzleVelocityField.getNorm();
        double nominalMuzzleSpeedMps = baseAvgRpm * p.rpmToMpsFactor;

        double scale = 1.0;
        if (nominalMuzzleSpeedMps > 1e-6) {
            scale = MathUtil.clamp(
                    requiredMuzzleSpeedMps / nominalMuzzleSpeedMps,
                    Constants.Shooter.movingShotScaleMin,
                    Constants.Shooter.movingShotScaleMax);
        }
        double actualMuzzleSpeedMps = Math.max(0.0, baseAvgRpm * scale * p.rpmToMpsFactor);

        double horizontalSpeedMps = actualMuzzleSpeedMps * Math.cos(Constants.ShooterSim.launchPitchRad);
        double verticalSpeedMps = actualMuzzleSpeedMps * Math.sin(Constants.ShooterSim.launchPitchRad);

        Translation2d muzzleVelocityField = new Translation2d(horizontalSpeedMps, new Rotation2d(yawFieldRad));
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
                0.0,
                0.0));

        double elapsed = 0.0;
        while (elapsed <= Constants.ShooterSim.maxFlightTimeSec + EVAL_DT_SEC) {
            simulator.update(EVAL_DT_SEC);
            var drained = simulator.drainCompletedShots();
            if (!drained.isEmpty()) {
                return drained.get(0);
            }
            elapsed += EVAL_DT_SEC;
        }

        return new ShotResult(false, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, 0.0,
                Constants.ShooterSim.maxFlightTimeSec);
    }

    private static List<ShotScenarioEvaluator.Scenario> buildScenarioSet() {
        Translation2d hub = Constants.Shooter.hubPositionField;
        List<ShotScenarioEvaluator.Scenario> list = new ArrayList<>();
        double[] distances = {1.4, 2.0, 2.6, 3.2, 3.8, 4.4, 5.0, 5.6, 6.2};
        double[] bearingsDeg = {-150, -120, -90, -60, -30, 0, 30, 60, 90, 120, 150, 180};
        double[] speeds = {0.0, 0.8, 1.4, 2.0};

        int idx = 0;
        for (double distance : distances) {
            for (double bearingDeg : bearingsDeg) {
                Rotation2d bearing = Rotation2d.fromDegrees(bearingDeg);
                Translation2d shooterPos = hub.minus(new Translation2d(distance, bearing));
                Pose2d robotPose = new Pose2d(
                        shooterPos.minus(Constants.Shooter.shooterOffsetRobot),
                        Rotation2d.fromDegrees(0.0));

                // still
                list.add(new ShotScenarioEvaluator.Scenario(
                        "s" + idx++,
                        robotPose,
                        new Translation2d()));

                // tangential velocities (both directions) and radial away
                Rotation2d tangentA = bearing.plus(Rotation2d.fromDegrees(90.0));
                Rotation2d tangentB = bearing.minus(Rotation2d.fromDegrees(90.0));
                for (double speed : speeds) {
                    if (speed <= 1e-6) {
                        continue;
                    }
                    list.add(new ShotScenarioEvaluator.Scenario(
                            "s" + idx++,
                            robotPose,
                            new Translation2d(speed, tangentA)));
                    list.add(new ShotScenarioEvaluator.Scenario(
                            "s" + idx++,
                            robotPose,
                            new Translation2d(speed, tangentB)));
                    list.add(new ShotScenarioEvaluator.Scenario(
                            "s" + idx++,
                            robotPose,
                            new Translation2d(speed, bearing)));
                }
            }
        }
        return list;
    }

    private static Point2D[] toPoints(double[] xs, double[] ys) {
        Point2D[] pts = new Point2D[xs.length];
        for (int i = 0; i < xs.length; i++) {
            pts[i] = new Point2D.Double(xs[i], ys[i]);
        }
        return pts;
    }

    private static void printRecommendation(Params p, Score score, int scenarioCount) {
        double[] top = new double[DISTANCE_POINTS_M.length];
        double[] bottom = new double[DISTANCE_POINTS_M.length];
        for (int i = 0; i < DISTANCE_POINTS_M.length; i++) {
            top[i] = p.avgRpm[i] + (TOP_BOTTOM_SPLIT_RPM[i] * 0.5);
            bottom[i] = p.avgRpm[i] - (TOP_BOTTOM_SPLIT_RPM[i] * 0.5);
        }

        System.out.printf("Best score over %d scenarios:%n", scenarioCount);
        System.out.printf("hitRate=%.3f | meanClosest=%.3f m | meanHorizontal=%.3f m | meanVertical=%.3f m%n",
                score.hitRate, score.meanClosest, score.meanHorizontal, score.meanVertical);
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
        System.out.println("Recommended flight-time points:");
        for (int i = 0; i < DISTANCE_POINTS_M.length; i++) {
            System.out.printf("  (%.1f, %.3f)%n", DISTANCE_POINTS_M[i], p.flightTimeSec[i]);
        }
    }
}
