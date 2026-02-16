package frc.robot.calibration;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.ShotKinematicSolver;

class CalibrationSessionTest {
    @Test
    void sessionAdvancesStepAndStoresLabeledSamples() {
        CalibrationPreset preset = new CalibrationPreset(
                "test",
                "test preset",
                List.of(
                        new CalibrationPreset.Step("step0", 2.0, new Translation2d(), 2),
                        new CalibrationPreset.Step("step1", 3.0, new Translation2d(), 1)));
        CalibrationSession session = new CalibrationSession(preset);

        CalibrationSample pending = new CalibrationSample(
                1.0,
                "test",
                0,
                "step0",
                2.0,
                new Pose2d(),
                new Translation2d(),
                2.0,
                3000.0,
                2800.0,
                0.0,
                ShotKinematicSolver.SolveStatus.SOLVED,
                false,
                null);

        session.registerFiredShot(pending);
        session.registerFiredShot(pending);

        assertEquals(1, session.getActiveStepIndex());
        assertEquals(2, session.getPendingSamplesCount());

        session.recordOutcome(ShotOutcome.HIT);
        session.recordOutcome(ShotOutcome.MISS_SHORT);

        assertEquals(2, session.getLabeledSampleCount());
        assertTrue(session.getLabeledSamples().stream().allMatch(s -> s.outcome() != null));
    }
}

