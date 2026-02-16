package frc.robot.calibration;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class CalibrationIOTest {
    @Test
    void validateAcceptsSortedFiniteArrays() {
        ShooterCalibrationConfig cfg = new ShooterCalibrationConfig();
        cfg.turretZeroOffsetRad = 0.01;
        cfg.biasDistancesM = new double[] {2.5, 3.5, 4.5};
        cfg.biasValues = new double[] {1.0, 1.05, 1.02};
        assertTrue(CalibrationIO.validate(cfg));
    }

    @Test
    void validateRejectsMismatchedArrayLengths() {
        ShooterCalibrationConfig cfg = new ShooterCalibrationConfig();
        cfg.biasDistancesM = new double[] {2.5, 3.5};
        cfg.biasValues = new double[] {1.0};
        assertFalse(CalibrationIO.validate(cfg));
    }
}

