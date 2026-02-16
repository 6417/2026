package frc.robot.calibration;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

/**
 * Deterministic parameter update from labeled field shots.
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
            double maxTurretOffsetDeg) {
    }

    public record FitResult(
            double turretZeroOffsetRad,
            double[] biasValues,
            int directionalSamples,
            int shortLongSamples) {
    }

    private CalibrationFitter() {
    }

    public static FitResult fit(
            List<CalibrationSample> samples,
            double currentTurretZeroOffsetRad,
            double[] baseBiasValues,
            Params params) {
        int knotCount = params.biasFitDistancesMeters().length;
        double[] weightedError = new double[knotCount];
        double[] weightSum = new double[knotCount];

        int leftMisses = 0;
        int rightMisses = 0;
        int directionalSamples = 0;
        int shortLongSamples = 0;

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

            double signedSpeedError = switch (sample.outcome()) {
                case MISS_SHORT -> 1.0;
                case MISS_LONG -> -1.0;
                default -> 0.0;
            };
            if (signedSpeedError == 0.0) {
                continue;
            }

            for (int i = 0; i < knotCount; i++) {
                double knotDistance = params.biasFitDistancesMeters()[i];
                double weight = triangularWeight(sample.distanceToHubMeters(), knotDistance, params.biasWeightWindowMeters());
                if (weight <= 0.0) {
                    continue;
                }
                weightedError[i] += signedSpeedError * weight;
                weightSum[i] += weight;
            }
        }

        double[] fittedBias = new double[knotCount];
        for (int i = 0; i < knotCount; i++) {
            double base = baseBiasValues[i];
            if (weightSum[i] <= 1e-9) {
                fittedBias[i] = base;
                continue;
            }
            double normalizedError = weightedError[i] / weightSum[i];
            double relativeStep = MathUtil.clamp(
                    params.biasLearningRate() * normalizedError,
                    -params.maxBiasRelativeStep(),
                    params.maxBiasRelativeStep());
            fittedBias[i] = MathUtil.clamp(base * (1.0 + relativeStep), params.minBiasValue(), params.maxBiasValue());
        }

        double turretOffset = currentTurretZeroOffsetRad;
        if (directionalSamples >= params.minDirectionalSamplesForTurretFit()) {
            double signedDirectionalMean = (rightMisses - leftMisses) / (double) directionalSamples;
            double correctionDeg = MathUtil.clamp(
                    signedDirectionalMean * params.turretDegPerDirectionalMiss(),
                    -params.maxTurretOffsetDeg(),
                    params.maxTurretOffsetDeg());
            turretOffset += Units.degreesToRadians(correctionDeg);
            double turretMaxAbs = Units.degreesToRadians(params.maxTurretOffsetDeg());
            turretOffset = MathUtil.clamp(turretOffset, -turretMaxAbs, turretMaxAbs);
        }

        return new FitResult(turretOffset, fittedBias, directionalSamples, shortLongSamples);
    }

    private static double triangularWeight(double sampleDistanceMeters, double knotDistanceMeters, double windowMeters) {
        if (windowMeters <= 0.0) {
            return 0.0;
        }
        double normalized = Math.abs(sampleDistanceMeters - knotDistanceMeters) / windowMeters;
        return Math.max(0.0, 1.0 - normalized);
    }
}

