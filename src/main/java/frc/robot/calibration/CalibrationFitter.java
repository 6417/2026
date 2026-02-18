package frc.robot.calibration;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/**
 * Deterministischer Fitter fuer Turret-Offset und Distanz-Bias aus gelabelten Feldschuessen.
 *
 * <p>Der Fitter kombiniert Label-Information (left/right/short/long) mit metrischen
 * Impact-Residualen (blau->rot). Updates werden bewusst begrenzt, damit der Fit
 * auch bei verrauschten Samples stabil bleibt.
 */
public final class CalibrationFitter {
    public record Params(
            double[] biasFitDistancesMeters,
            double biasWeightWindowMeters,
            double biasLearningRate,
            double maxBiasRelativeStep,
            double minBiasValue,
            double maxBiasValue,
            int minDirectionalSamplesForTurretFit,
            double turretDegPerDirectionalMiss,
            double maxTurretOffsetDeg,
            double impactResidualWeight,
            double impactRangeMetersPerBiasUnit,
            double impactLateralMetersPerTurretDeg,
            double turretResidualLearningRate,
            double biasResidualLearningRate,
            double maxTurretDeltaDegPerRound,
            double maxBiasDeltaPerRound,
            double minLateralWeightForTurretUpdate,
            double minRangeWeightForBiasUpdate,
            double fitShrinkFactorLowConfidence,
            double residualHuberDeltaMeters) {
    }

    public record FitResult(
            double turretZeroOffsetRad,
            double[] biasValues,
            int directionalSamples,
            int shortLongSamples,
            double turretDeltaRad,
            boolean turretUpdateSuppressed,
            double lateralWeightUsed,
            double[] rangeWeightPerKnot,
            int suppressedBiasKnots) {
    }

    public record ResidualProjection(
            double rangeResidualMeters,
            double lateralResidualMeters,
            boolean valid) {
    }

    private CalibrationFitter() {
    }

    /**
     * Fuehrt einen vollstaendigen Fit ueber die gelabelten Samples aus.
     */
    public static FitResult fit(
            List<CalibrationSample> samples,
            double currentTurretZeroOffsetRad,
            double[] baseBiasValues,
            Params params) {
        int knotCount = params.biasFitDistancesMeters().length;
        double[] weightedSignal = new double[knotCount];
        double[] rangeWeightPerKnot = new double[knotCount];

        int leftMisses = 0;
        int rightMisses = 0;
        int directionalSamples = 0;
        int shortLongSamples = 0;
        double weightedLateralResidualMeters = 0.0;
        double weightedLateralWeight = 0.0;

        for (CalibrationSample sample : samples) {
            if (sample.outcome() == null) {
                continue;
            }
            switch (sample.outcome()) {
                case MISS_LEFT -> {
                    leftMisses++;
                    directionalSamples++;
                }
                case MISS_RIGHT -> {
                    rightMisses++;
                    directionalSamples++;
                }
                case MISS_SHORT, MISS_LONG -> shortLongSamples++;
                default -> {
                    // no-op
                }
            }

            double labelSignal = switch (sample.outcome()) {
                case MISS_SHORT -> 1.0;
                case MISS_LONG -> -1.0;
                default -> 0.0;
            };

            double residualSignal = 0.0;
            double residualRangeWeight = 0.0;
            double residualLateralWeight = 0.0;
            double lateralResidualMeters = 0.0;

            if (sample.predictedImpactField() != null && sample.adjustedImpactField() != null && sample.impactAdjusted()) {
                Translation2d shooter = sample.robotPoseField().getTranslation()
                        .plus(Constants.Shooter.shooterOffsetRobot.rotateBy(sample.robotPoseField().getRotation()));
                ResidualProjection projection = projectImpactResidual(
                        sample.predictedImpactField(),
                        sample.adjustedImpactField(),
                        shooter,
                        Constants.Shooter.hubPositionField);
                if (projection.valid()) {
                    double confidence = MathUtil.clamp(sample.impactAdjustConfidence(), 0.0, 1.0);
                    double robustRangeWeight = huberWeight(projection.rangeResidualMeters(), params.residualHuberDeltaMeters());
                    double robustLateralWeight = huberWeight(projection.lateralResidualMeters(), params.residualHuberDeltaMeters());
                    residualRangeWeight = params.impactResidualWeight() * confidence * robustRangeWeight;
                    residualLateralWeight = params.impactResidualWeight() * confidence * robustLateralWeight;
                    if (params.impactRangeMetersPerBiasUnit() > 1e-9) {
                        // Negative sign is intentional:
                        // if actual impact is short of predicted (negative residual), we need more speed/bias.
                        residualSignal = (-projection.rangeResidualMeters() / params.impactRangeMetersPerBiasUnit())
                                * params.biasResidualLearningRate();
                    }
                    lateralResidualMeters = projection.lateralResidualMeters();
                }
            }

            for (int i = 0; i < knotCount; i++) {
                double knotDistance = params.biasFitDistancesMeters()[i];
                double weight = triangularWeight(sample.distanceToHubMeters(), knotDistance, params.biasWeightWindowMeters());
                if (weight <= 0.0) {
                    continue;
                }

                if (Math.abs(labelSignal) > 1e-9) {
                    weightedSignal[i] += labelSignal * weight;
                    rangeWeightPerKnot[i] += weight;
                }
                if (Math.abs(residualSignal) > 1e-9 && residualRangeWeight > 0.0) {
                    double combinedWeight = weight * residualRangeWeight;
                    weightedSignal[i] += residualSignal * combinedWeight;
                    rangeWeightPerKnot[i] += combinedWeight;
                }
            }

            if (Math.abs(lateralResidualMeters) > 1e-9 && residualLateralWeight > 0.0) {
                weightedLateralResidualMeters += lateralResidualMeters * residualLateralWeight;
                weightedLateralWeight += residualLateralWeight;
            }
        }

        double[] fittedBias = new double[knotCount];
        int suppressedBiasKnots = 0;
        for (int i = 0; i < knotCount; i++) {
            double base = baseBiasValues[i];
            if (rangeWeightPerKnot[i] <= 1e-9) {
                fittedBias[i] = base;
                suppressedBiasKnots++;
                continue;
            }

            double normalizedSignal = weightedSignal[i] / rangeWeightPerKnot[i];
            double confidenceScale = lowConfidenceScale(
                    rangeWeightPerKnot[i],
                    params.minRangeWeightForBiasUpdate(),
                    params.fitShrinkFactorLowConfidence());
            if (rangeWeightPerKnot[i] < params.minRangeWeightForBiasUpdate()) {
                suppressedBiasKnots++;
            }

            double relativeStep = MathUtil.clamp(params.biasLearningRate() * normalizedSignal * confidenceScale,
                    -params.maxBiasRelativeStep(),
                    params.maxBiasRelativeStep());
            double absoluteDelta = MathUtil.clamp(
                    base * relativeStep,
                    -params.maxBiasDeltaPerRound(),
                    params.maxBiasDeltaPerRound());
            fittedBias[i] = MathUtil.clamp(base + absoluteDelta, params.minBiasValue(), params.maxBiasValue());
        }

        double turretOffset = currentTurretZeroOffsetRad;
        double turretDeltaDeg = 0.0;
        boolean usedDirectional = false;
        boolean usedResidual = false;

        if (directionalSamples >= params.minDirectionalSamplesForTurretFit()) {
            double signedDirectionalMean = (rightMisses - leftMisses) / (double) directionalSamples;
            double correctionDeg = MathUtil.clamp(
                    signedDirectionalMean * params.turretDegPerDirectionalMiss(),
                    -params.maxTurretDeltaDegPerRound(),
                    params.maxTurretDeltaDegPerRound());
            turretDeltaDeg += correctionDeg;
            usedDirectional = true;
        }

        if (weightedLateralWeight > 1e-9
                && params.impactLateralMetersPerTurretDeg() > 1e-9
                && weightedLateralWeight >= params.minLateralWeightForTurretUpdate()) {
            double meanLateralResidual = weightedLateralResidualMeters / weightedLateralWeight;
            double confidenceScale = lowConfidenceScale(
                    weightedLateralWeight,
                    params.minLateralWeightForTurretUpdate(),
                    params.fitShrinkFactorLowConfidence());
            double correctionDeg = MathUtil.clamp(
                    (meanLateralResidual / params.impactLateralMetersPerTurretDeg())
                            * params.impactResidualWeight()
                            * params.turretResidualLearningRate()
                            * confidenceScale,
                    -params.maxTurretDeltaDegPerRound(),
                    params.maxTurretDeltaDegPerRound());
            turretDeltaDeg += correctionDeg;
            usedResidual = true;
        }

        turretDeltaDeg = MathUtil.clamp(
                turretDeltaDeg,
                -params.maxTurretDeltaDegPerRound(),
                params.maxTurretDeltaDegPerRound());
        turretOffset += Units.degreesToRadians(turretDeltaDeg);

        double turretMaxAbs = Units.degreesToRadians(params.maxTurretOffsetDeg());
        turretOffset = MathUtil.clamp(turretOffset, -turretMaxAbs, turretMaxAbs);

        boolean turretSuppressed = !usedDirectional && !usedResidual;

        return new FitResult(
                turretOffset,
                fittedBias,
                directionalSamples,
                shortLongSamples,
                Units.degreesToRadians(turretDeltaDeg),
                turretSuppressed,
                weightedLateralWeight,
                Arrays.copyOf(rangeWeightPerKnot, rangeWeightPerKnot.length),
                suppressedBiasKnots);
    }

    static ResidualProjection projectImpactResidual(
            Translation2d predictedImpactField,
            Translation2d adjustedImpactField,
            Translation2d shooterField,
            Translation2d hubField) {
        Translation2d forward = hubField.minus(shooterField);
        double norm = forward.getNorm();
        if (norm <= 1e-6) {
            forward = predictedImpactField.minus(shooterField);
            norm = forward.getNorm();
        }
        if (norm <= 1e-6) {
            return new ResidualProjection(0.0, 0.0, false);
        }

        forward = new Translation2d(forward.getX() / norm, forward.getY() / norm);
        Translation2d right = new Translation2d(forward.getY(), -forward.getX());
        Translation2d residual = adjustedImpactField.minus(predictedImpactField);

        double rangeResidual = residual.getX() * forward.getX() + residual.getY() * forward.getY();
        double lateralResidual = residual.getX() * right.getX() + residual.getY() * right.getY();
        return new ResidualProjection(rangeResidual, lateralResidual, true);
    }

    private static double huberWeight(double residual, double deltaMeters) {
        if (deltaMeters <= 1e-9) {
            return 1.0;
        }
        double abs = Math.abs(residual);
        if (abs <= deltaMeters) {
            return 1.0;
        }
        return deltaMeters / abs;
    }

    private static double lowConfidenceScale(double weight, double minWeight, double shrinkFactor) {
        if (minWeight <= 1e-9 || weight >= minWeight) {
            return 1.0;
        }
        double ratio = MathUtil.clamp(weight / minWeight, 0.0, 1.0);
        return MathUtil.clamp(shrinkFactor * ratio, 0.0, 1.0);
    }

    /**
     * Triangulaere Gewichtung um einen Knotenpunkt.
     *
     * <p>Samples nahe am Knoten beeinflussen ihn stark, weit entfernte schwach bis gar nicht.
     */
    private static double triangularWeight(double sampleDistanceMeters, double knotDistanceMeters, double windowMeters) {
        if (windowMeters <= 0.0) {
            return 0.0;
        }
        double normalized = Math.abs(sampleDistanceMeters - knotDistanceMeters) / windowMeters;
        return Math.max(0.0, 1.0 - normalized);
    }
}
