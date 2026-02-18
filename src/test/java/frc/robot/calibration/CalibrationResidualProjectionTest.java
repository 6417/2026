package frc.robot.calibration;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;

class CalibrationResidualProjectionTest {
    @Test
    void projectionUsesHubAlignedFrameSigns() {
        Translation2d shooter = new Translation2d(0.0, 0.0);
        Translation2d hub = new Translation2d(10.0, 0.0);
        Translation2d predicted = new Translation2d(4.0, 0.5);
        Translation2d adjusted = new Translation2d(5.0, 1.5);

        CalibrationFitter.ResidualProjection projection = CalibrationFitter.projectImpactResidual(
                predicted,
                adjusted,
                shooter,
                hub);

        assertTrue(projection.valid());
        assertEquals(1.0, projection.rangeResidualMeters(), 1e-9);
        assertEquals(-1.0, projection.lateralResidualMeters(), 1e-9);
    }

    @Test
    void projectionRotatesWithField() {
        Translation2d shooter = new Translation2d(0.0, 0.0);
        Translation2d hub = new Translation2d(0.0, 10.0);
        Translation2d predicted = new Translation2d(0.5, 4.0);
        Translation2d adjusted = new Translation2d(1.5, 5.0);

        CalibrationFitter.ResidualProjection projection = CalibrationFitter.projectImpactResidual(
                predicted,
                adjusted,
                shooter,
                hub);

        assertTrue(projection.valid());
        assertEquals(1.0, projection.rangeResidualMeters(), 1e-9);
        assertEquals(1.0, projection.lateralResidualMeters(), 1e-9);
    }

    @Test
    void projectionInvalidWhenFrameUndefined() {
        Translation2d same = new Translation2d(1.0, 1.0);
        CalibrationFitter.ResidualProjection projection = CalibrationFitter.projectImpactResidual(
                same,
                same,
                same,
                same);

        assertFalse(projection.valid());
    }
}
