package frc.robot.sim;

import java.awt.geom.Point2D;
import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.utils.LinearInterpolationTable;
import frc.robot.utils.ShotKinematicSolver;

/**
 * Offline evaluator for shooter/turret scenarios.
 *
 * <p>This class intentionally mirrors {@code ShooterSubsystem#calculateShotCommand}. Keeping both
 * paths aligned is mandatory so offline plots and runtime behavior match.
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
            double requiredMuzzleSpeedMps,
            double muzzleSpeedMps,
            double closestDistanceMeters,
            double horizontalErrorMeters,
            double verticalErrorMeters,
            double flightTimeSec,
            ShotKinematicSolver.SolveStatus solveStatus,
            boolean rpmSaturatedTop,
            boolean rpmSaturatedBottom) {
        public boolean rpmSaturatedAny() {
            return rpmSaturatedTop || rpmSaturatedBottom;
        }
    }

    /**
     * Parameterized evaluation config used for deterministic tuning sweeps.
     *
     * <p>Default config comes from current constants, and optional `with*` helpers allow local
     * override without touching code constants.
     */
    public record EvaluationConfig(
            Translation2d hubPositionField,
            Translation2d shooterOffsetRobot,
            double turretZeroOnRobotRad,
            LinearInterpolationTable topRpmTable,
            LinearInterpolationTable bottomRpmTable,
            LinearInterpolationTable distanceScaleBiasTable,
            double rpmToMpsFactor,
            double maxRpm,
            double launchHeightMeters,
            double hubCenterHeightMeters,
            double hubRadiusMeters,
            double hubHeightToleranceMeters,
            double launchPitchRad,
            double gravityMetersPerSec2,
            double maxFlightTimeSec,
            double ballisticDragCoeffPerMeter,
            double solverDragCoeffPerMeter,
            double solverDragSpeedCompensationGain,
            double solverMinFlightTimeSec,
            double solverMaxFlightTimeSec,
            int solverMaxIterations,
            double solverFlightTimeToleranceSec,
            double solverVerticalErrorToleranceMeters) {

        public static EvaluationConfig fromConstants() {
            return new EvaluationConfig(
                    Constants.Shooter.hubPositionField,
                    Constants.Shooter.shooterOffsetRobot,
                    Constants.Shooter.turretZeroOnRobotRad,
                    Constants.Shooter.topRpmTable,
                    Constants.Shooter.bottomRpmTable,
                    Constants.Shooter.distanceScaleBiasTable,
                    Constants.Shooter.rpmToMpsFactor,
                    Constants.Shooter.maxRpm,
                    Constants.ShooterSim.launchHeightMeters,
                    Constants.ShooterSim.hubCenterHeightMeters,
                    Constants.ShooterSim.hubRadiusMeters,
                    Constants.ShooterSim.hubHeightToleranceMeters,
                    Constants.ShooterSim.launchPitchRad,
                    Constants.ShooterSim.gravityMetersPerSec2,
                    Constants.ShooterSim.maxFlightTimeSec,
                    Constants.ShooterSim.dragCoefficientPerMeter,
                    Constants.ShooterSim.dragCoefficientPerMeter,
                    Constants.Shooter.solverDragSpeedCompensationGain,
                    Constants.Shooter.solverMinFlightTimeSec,
                    Constants.Shooter.solverMaxFlightTimeSec,
                    Constants.Shooter.solverMaxIterations,
                    Constants.Shooter.solverFlightTimeToleranceSec,
                    Constants.Shooter.solverVerticalErrorToleranceMeters);
        }

        public EvaluationConfig withDistanceScaleBiasPoints(Point2D[] points) {
            return new EvaluationConfig(
                    hubPositionField,
                    shooterOffsetRobot,
                    turretZeroOnRobotRad,
                    topRpmTable,
                    bottomRpmTable,
                    new LinearInterpolationTable(points),
                    rpmToMpsFactor,
                    maxRpm,
                    launchHeightMeters,
                    hubCenterHeightMeters,
                    hubRadiusMeters,
                    hubHeightToleranceMeters,
                    launchPitchRad,
                    gravityMetersPerSec2,
                    maxFlightTimeSec,
                    ballisticDragCoeffPerMeter,
                    solverDragCoeffPerMeter,
                    solverDragSpeedCompensationGain,
                    solverMinFlightTimeSec,
                    solverMaxFlightTimeSec,
                    solverMaxIterations,
                    solverFlightTimeToleranceSec,
                    solverVerticalErrorToleranceMeters);
        }

        public EvaluationConfig withRpmToMpsFactor(double value) {
            return new EvaluationConfig(
                    hubPositionField,
                    shooterOffsetRobot,
                    turretZeroOnRobotRad,
                    topRpmTable,
                    bottomRpmTable,
                    distanceScaleBiasTable,
                    value,
                    maxRpm,
                    launchHeightMeters,
                    hubCenterHeightMeters,
                    hubRadiusMeters,
                    hubHeightToleranceMeters,
                    launchPitchRad,
                    gravityMetersPerSec2,
                    maxFlightTimeSec,
                    ballisticDragCoeffPerMeter,
                    solverDragCoeffPerMeter,
                    solverDragSpeedCompensationGain,
                    solverMinFlightTimeSec,
                    solverMaxFlightTimeSec,
                    solverMaxIterations,
                    solverFlightTimeToleranceSec,
                    solverVerticalErrorToleranceMeters);
        }

        public EvaluationConfig withSolverDragSpeedCompensationGain(double value) {
            return new EvaluationConfig(
                    hubPositionField,
                    shooterOffsetRobot,
                    turretZeroOnRobotRad,
                    topRpmTable,
                    bottomRpmTable,
                    distanceScaleBiasTable,
                    rpmToMpsFactor,
                    maxRpm,
                    launchHeightMeters,
                    hubCenterHeightMeters,
                    hubRadiusMeters,
                    hubHeightToleranceMeters,
                    launchPitchRad,
                    gravityMetersPerSec2,
                    maxFlightTimeSec,
                    ballisticDragCoeffPerMeter,
                    solverDragCoeffPerMeter,
                    value,
                    solverMinFlightTimeSec,
                    solverMaxFlightTimeSec,
                    solverMaxIterations,
                    solverFlightTimeToleranceSec,
                    solverVerticalErrorToleranceMeters);
        }

        public EvaluationConfig withSolverDragCoeffPerMeter(double value) {
            return new EvaluationConfig(
                    hubPositionField,
                    shooterOffsetRobot,
                    turretZeroOnRobotRad,
                    topRpmTable,
                    bottomRpmTable,
                    distanceScaleBiasTable,
                    rpmToMpsFactor,
                    maxRpm,
                    launchHeightMeters,
                    hubCenterHeightMeters,
                    hubRadiusMeters,
                    hubHeightToleranceMeters,
                    launchPitchRad,
                    gravityMetersPerSec2,
                    maxFlightTimeSec,
                    ballisticDragCoeffPerMeter,
                    value,
                    solverDragSpeedCompensationGain,
                    solverMinFlightTimeSec,
                    solverMaxFlightTimeSec,
                    solverMaxIterations,
                    solverFlightTimeToleranceSec,
                    solverVerticalErrorToleranceMeters);
        }

        public EvaluationConfig withBallisticDragCoeffPerMeter(double value) {
            return new EvaluationConfig(
                    hubPositionField,
                    shooterOffsetRobot,
                    turretZeroOnRobotRad,
                    topRpmTable,
                    bottomRpmTable,
                    distanceScaleBiasTable,
                    rpmToMpsFactor,
                    maxRpm,
                    launchHeightMeters,
                    hubCenterHeightMeters,
                    hubRadiusMeters,
                    hubHeightToleranceMeters,
                    launchPitchRad,
                    gravityMetersPerSec2,
                    maxFlightTimeSec,
                    value,
                    solverDragCoeffPerMeter,
                    solverDragSpeedCompensationGain,
                    solverMinFlightTimeSec,
                    solverMaxFlightTimeSec,
                    solverMaxIterations,
                    solverFlightTimeToleranceSec,
                    solverVerticalErrorToleranceMeters);
        }
    }

    private static final double EVAL_DT_SEC = 0.005;

    public Evaluation evaluate(Scenario scenario) {
        return evaluate(scenario, EvaluationConfig.fromConstants());
    }

    public Evaluation evaluate(Scenario scenario, EvaluationConfig config) {
        Translation2d shooterPositionField = scenario.robotPoseField().getTranslation().plus(
                config.shooterOffsetRobot().rotateBy(scenario.robotPoseField().getRotation()));
        Translation2d shooterToHubField = config.hubPositionField().minus(shooterPositionField);
        double distanceMeters = shooterToHubField.getNorm();
        if (distanceMeters < 1e-6) {
            return invalidEvaluation(ShotKinematicSolver.SolveStatus.TARGET_TOO_CLOSE);
        }

        double baseTopRpm = config.topRpmTable().getOutput(distanceMeters);
        double baseBottomRpm = config.bottomRpmTable().getOutput(distanceMeters);
        double splitRpm = baseTopRpm - baseBottomRpm;

        ShotKinematicSolver.SolveResult solveResult = ShotKinematicSolver.solve(
                new ShotKinematicSolver.SolveRequest(
                        new Translation3d(
                                shooterPositionField.getX(),
                                shooterPositionField.getY(),
                                config.launchHeightMeters()),
                        new Translation3d(
                                config.hubPositionField().getX(),
                                config.hubPositionField().getY(),
                                config.hubCenterHeightMeters()),
                        scenario.robotVelocityFieldMps(),
                        config.launchPitchRad(),
                        config.gravityMetersPerSec2(),
                        config.solverMinFlightTimeSec(),
                        config.solverMaxFlightTimeSec(),
                        config.solverMaxIterations(),
                        config.solverFlightTimeToleranceSec(),
                        config.solverVerticalErrorToleranceMeters(),
                        config.solverDragCoeffPerMeter(),
                        config.solverDragSpeedCompensationGain()));

        if (!solveResult.solved()) {
            return invalidEvaluation(solveResult.status());
        }

        double turretYawRobotRad = MathUtil.angleModulus(
                solveResult.yawFieldRad()
                        - scenario.robotPoseField().getRotation().getRadians()
                        - config.turretZeroOnRobotRad());

        double requiredAvgRpm = solveResult.requiredMuzzleSpeedMps() / config.rpmToMpsFactor();
        requiredAvgRpm *= config.distanceScaleBiasTable().getOutput(distanceMeters);
        if (!Double.isFinite(requiredAvgRpm)) {
            return invalidEvaluation(ShotKinematicSolver.SolveStatus.INVALID_INPUT);
        }

        double requestedTopRpm = requiredAvgRpm + (splitRpm * 0.5);
        double requestedBottomRpm = requiredAvgRpm - (splitRpm * 0.5);
        RpmLimitResult topLimit = clampRpmWithStatus(requestedTopRpm, config.maxRpm());
        RpmLimitResult bottomLimit = clampRpmWithStatus(requestedBottomRpm, config.maxRpm());

        double actualMuzzleSpeedMps = Math.max(
                0.0,
                ((topLimit.rpm() + bottomLimit.rpm()) * 0.5) * config.rpmToMpsFactor());
        double horizontalSpeedMps = actualMuzzleSpeedMps * Math.cos(config.launchPitchRad());
        double verticalSpeedMps = actualMuzzleSpeedMps * Math.sin(config.launchPitchRad());

        Translation2d muzzleVelocityField = new Translation2d(horizontalSpeedMps, solveResult.muzzleVelocityFieldMps().getAngle());
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
                new Translation3d(
                        shooterPositionField.getX(),
                        shooterPositionField.getY(),
                        config.launchHeightMeters()),
                new Translation3d(
                        initialVelocityField.getX(),
                        initialVelocityField.getY(),
                        verticalSpeedMps),
                turretYawRobotRad,
                topLimit.rpm(),
                bottomLimit.rpm()));

        Optional<ShotResult> result = Optional.empty();
        double elapsed = 0.0;
        while (elapsed <= config.maxFlightTimeSec() + EVAL_DT_SEC) {
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
                    topLimit.rpm(),
                    bottomLimit.rpm(),
                    solveResult.requiredMuzzleSpeedMps(),
                    actualMuzzleSpeedMps,
                    Double.POSITIVE_INFINITY,
                    Double.POSITIVE_INFINITY,
                    Double.POSITIVE_INFINITY,
                    config.maxFlightTimeSec(),
                    solveResult.status(),
                    topLimit.saturated(),
                    bottomLimit.saturated());
        }

        ShotResult shotResult = result.get();
        return new Evaluation(
                shotResult.hit(),
                Math.toDegrees(turretYawRobotRad),
                topLimit.rpm(),
                bottomLimit.rpm(),
                solveResult.requiredMuzzleSpeedMps(),
                actualMuzzleSpeedMps,
                shotResult.closestDistanceMeters(),
                shotResult.horizontalErrorMeters(),
                shotResult.verticalErrorMeters(),
                shotResult.flightTimeSec(),
                solveResult.status(),
                topLimit.saturated(),
                bottomLimit.saturated());
    }

    private Evaluation invalidEvaluation(ShotKinematicSolver.SolveStatus status) {
        return new Evaluation(
                false,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                Double.POSITIVE_INFINITY,
                Double.POSITIVE_INFINITY,
                Double.POSITIVE_INFINITY,
                0.0,
                status,
                false,
                false);
    }

    private RpmLimitResult clampRpmWithStatus(double rpm, double maxRpm) {
        if (maxRpm > 0.0) {
            double clamped = MathUtil.clamp(rpm, -maxRpm, maxRpm);
            boolean saturated = Math.abs(clamped - rpm) > 1e-6;
            return new RpmLimitResult(clamped, saturated);
        }
        return new RpmLimitResult(rpm, false);
    }

    private record RpmLimitResult(double rpm, boolean saturated) {
    }
}
