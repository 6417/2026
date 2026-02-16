package frc.robot.calibration;

import java.util.Arrays;

/**
 * Persistierbares Datenmodell für Shooter-Calibration-Overrides.
 *
 * <p>Diese Werte übersteuern zur Laufzeit die statischen Konstanten:
 * - Turret-Nulloffset
 * - Distanzabhängige Bias-Kurve (Knoten + Werte)
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

    /** @return tiefe Kopie, um Seiteneffekte beim Speichern/Anwenden zu vermeiden. */
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

