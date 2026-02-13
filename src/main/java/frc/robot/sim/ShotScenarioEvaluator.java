package frc.robot.sim;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;

/**
 * Offline evaluator for shooter/turret scenarios.
 *
 * <p>This lets us provide robot pose + velocity and measure predicted shot quality
 * against the hub using the same constants/tables as runtime logic.
 *
 * <p>Important: keep this math aligned with ShooterSubsystem#calculateShotCommand.
 * Any mismatch here makes offline tuning disagree with live robot behavior.
 */
public class ShotScenarioEvaluator {

    public record Scenario(
            String name,
            Pose2d robotPoseField,
            Translation2d robotVelocityFieldMps) {
    }

    public record Evaluation(
            boolean hit,
            double turretYawRobotDeg,
            double topRpm,
            double bottomRpm,
            double muzzleSpeedMps,
            double closestDistanceMeters,
            double horizontalErrorMeters,
            double verticalErrorMeters,
            double flightTimeSec) {
    }

    private static final double EVAL_DT_SEC = 0.005;

    public Evaluation evaluate(Scenario scenario) {
        Translation2d shooterPositionField = scenario.robotPoseField().getTranslation().plus(
                Constants.Shooter.shooterOffsetRobot.rotateBy(scenario.robotPoseField().getRotation()));
        Translation2d shooterToHubField = Constants.Shooter.hubPositionField.minus(shooterPositionField);
        double distanceMeters = shooterToHubField.getNorm();
        if (distanceMeters < 1e-6) {
            return new Evaluation(false, 0.0, 0.0, 0.0, 0.0, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY,
                    Double.POSITIVE_INFINITY, 0.0);
        }

        double baseTopRpm = Constants.Shooter.topRpmTable.getOutput(distanceMeters);
        double baseBottomRpm = Constants.Shooter.bottomRpmTable.getOutput(distanceMeters);
        double flightTimeEstimateSec = Math.max(
                Constants.Shooter.minFlightTimeSec,
                Constants.Shooter.flightTimeTable.getOutput(distanceMeters));

        Translation2d requiredBallVelocityField = shooterToHubField.div(flightTimeEstimateSec);
        Translation2d requiredMuzzleVelocityField = requiredBallVelocityField.minus(scenario.robotVelocityFieldMps());

        double yawFieldRad = requiredMuzzleVelocityField.getAngle().getRadians();
        double turretYawRobotRad = MathUtil.angleModulus(
                yawFieldRad - scenario.robotPoseField().getRotation().getRadians() - Constants.Shooter.turretZeroOnRobotRad);

        double requiredMuzzleSpeedMps = requiredMuzzleVelocityField.getNorm();
        double nominalMuzzleSpeedMps = ((baseTopRpm + baseBottomRpm) * 0.5) * Constants.Shooter.rpmToMpsFactor;

        double rawScale = 1.0;
        if (nominalMuzzleSpeedMps > 1e-6) {
            rawScale = requiredMuzzleSpeedMps / nominalMuzzleSpeedMps;
        }
        // Distance bias is a smooth correction term used to reduce clamp saturation
        // without forcing abrupt RPM table changes.
        rawScale *= Constants.Shooter.getDistanceScaleBias(distanceMeters);
        double scale = MathUtil.clamp(rawScale, Constants.Shooter.movingShotScaleMin, Constants.Shooter.movingShotScaleMax);

        double correctedTopRpm = clampRpm(baseTopRpm * scale);
        double correctedBottomRpm = clampRpm(baseBottomRpm * scale);
        double actualMuzzleSpeedMps = Math.max(
                0.0,
                ((correctedTopRpm + correctedBottomRpm) * 0.5) * Constants.Shooter.rpmToMpsFactor);

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
                Constants.ShooterSim.dragCoefficientPerMeter);

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
                turretYawRobotRad,
                correctedTopRpm,
                correctedBottomRpm));

        Optional<ShotResult> result = Optional.empty();
        double elapsed = 0.0;
        while (elapsed <= Constants.ShooterSim.maxFlightTimeSec + EVAL_DT_SEC) {
            simulator.update(EVAL_DT_SEC);
            var drained = simulator.drainCompletedShots();
            if (!drained.isEmpty()) {
                result = Optional.of(drained.get(0));
                break;
            }
            elapsed += EVAL_DT_SEC;
        }

        if (result.isEmpty()) {
            return new Evaluation(
                    false,
                    Math.toDegrees(turretYawRobotRad),
                    correctedTopRpm,
                    correctedBottomRpm,
                    actualMuzzleSpeedMps,
                    Double.POSITIVE_INFINITY,
                    Double.POSITIVE_INFINITY,
                    Double.POSITIVE_INFINITY,
                    Constants.ShooterSim.maxFlightTimeSec);
        }

        ShotResult shotResult = result.get();
        return new Evaluation(
                shotResult.hit(),
                Math.toDegrees(turretYawRobotRad),
                correctedTopRpm,
                correctedBottomRpm,
                actualMuzzleSpeedMps,
                shotResult.closestDistanceMeters(),
                shotResult.horizontalErrorMeters(),
                shotResult.verticalErrorMeters(),
                shotResult.flightTimeSec());
    }

    private double clampRpm(double rpm) {
        if (Constants.Shooter.maxRpm > 0.0) {
            return MathUtil.clamp(rpm, -Constants.Shooter.maxRpm, Constants.Shooter.maxRpm);
        }
        return rpm;
    }
}
