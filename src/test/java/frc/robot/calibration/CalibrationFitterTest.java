package frc.robot.calibration;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.ShotKinematicSolver;

class CalibrationFitterTest {
    private static CalibrationSample sample(double distanceMeters, ShotOutcome outcome) {
        return new CalibrationSample(
                1.0,
                "test",
                0,
                "s",
                distanceMeters,
                new Pose2d(),
                new Translation2d(),
                distanceMeters,
                3000.0,
                2800.0,
                0.0,
                ShotKinematicSolver.SolveStatus.SOLVED,
                false,
                outcome);
    }

    @Test
    void missShortIncreasesLocalBias() {
        double[] knots = {2.5, 3.5, 4.5, 5.5};
        double[] baseBias = {1.0, 1.0, 1.0, 1.0};

        CalibrationFitter.FitResult fit = CalibrationFitter.fit(
                List.of(sample(3.5, ShotOutcome.MISS_SHORT), sample(3.4, ShotOutcome.MISS_SHORT)),
                0.0,
                baseBias,
                new CalibrationFitter.Params(knots, 1.0, 0.12, 0.15, 0.80, 1.35, 4, 2.0, 4.0));

        assertTrue(fit.biasValues()[1] > 1.0);
    }

    @Test
    void directionalMissesAdjustTurretOffset() {
        double[] knots = {2.5, 3.5, 4.5, 5.5};
        double[] baseBias = {1.0, 1.0, 1.0, 1.0};

        CalibrationFitter.FitResult fit = CalibrationFitter.fit(
                List.of(
                        sample(3.0, ShotOutcome.MISS_RIGHT),
                        sample(3.0, ShotOutcome.MISS_RIGHT),
                        sample(3.0, ShotOutcome.MISS_RIGHT),
                        sample(3.0, ShotOutcome.MISS_RIGHT)),
                0.0,
                baseBias,
                new CalibrationFitter.Params(knots, 1.0, 0.12, 0.15, 0.80, 1.35, 4, 2.0, 4.0));

        assertTrue(fit.turretZeroOffsetRad() > 0.0);
    }
}

