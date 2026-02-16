package frc.robot.calibration;

import java.util.Arrays;

/**
 * Persisted runtime override for shooter calibration values.
 */
public class ShooterCalibrationConfig {
    public int version = 1;
    public double turretZeroOffsetRad = 0.0;
    public double[] biasDistancesM = new double[0];
    public double[] biasValues = new double[0];
    public int sampleCount = 0;
    public String source = "unknown";
    public String generatedAtIsoUtc = "";

    public ShooterCalibrationConfig() {
    }

    public ShooterCalibrationConfig copy() {
        ShooterCalibrationConfig clone = new ShooterCalibrationConfig();
        clone.version = version;
        clone.turretZeroOffsetRad = turretZeroOffsetRad;
        clone.biasDistancesM = Arrays.copyOf(biasDistancesM, biasDistancesM.length);
        clone.biasValues = Arrays.copyOf(biasValues, biasValues.length);
        clone.sampleCount = sampleCount;
        clone.source = source;
        clone.generatedAtIsoUtc = generatedAtIsoUtc;
        return clone;
    }
}

