package frc.robot.calibration;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.Optional;
import java.util.UUID;

/**
 * Mutable state for one calibration run.
 */
public class CalibrationSession {
    private final String sessionId = UUID.randomUUID().toString();
    private final CalibrationPreset preset;

    private int activeStepIndex = 0;
    private int shotsFiredInStep = 0;

    private final Deque<CalibrationSample> pendingSamples = new ArrayDeque<>();
    private final List<CalibrationSample> labeledSamples = new ArrayList<>();

    public CalibrationSession(CalibrationPreset preset) {
        this.preset = preset;
    }

    public String getSessionId() {
        return sessionId;
    }

    public CalibrationPreset getPreset() {
        return preset;
    }

    public int getActiveStepIndex() {
        return activeStepIndex;
    }

    public int getShotsFiredInStep() {
        return shotsFiredInStep;
    }

    public int getPendingSamplesCount() {
        return pendingSamples.size();
    }

    public int getLabeledSampleCount() {
        return labeledSamples.size();
    }

    public Optional<CalibrationPreset.Step> getActiveStep() {
        if (activeStepIndex < 0 || activeStepIndex >= preset.steps().size()) {
            return Optional.empty();
        }
        return Optional.of(preset.steps().get(activeStepIndex));
    }

    public boolean isComplete() {
        return activeStepIndex >= preset.steps().size();
    }

    public void registerFiredShot(CalibrationSample pendingSample) {
        if (isComplete()) {
            return;
        }
        pendingSamples.addLast(pendingSample.withOutcome(null));
        shotsFiredInStep++;

        CalibrationPreset.Step step = preset.steps().get(activeStepIndex);
        if (shotsFiredInStep >= Math.max(1, step.shotsRequired())) {
            activeStepIndex++;
            shotsFiredInStep = 0;
        }
    }

    public Optional<CalibrationSample> recordOutcome(ShotOutcome outcome) {
        CalibrationSample pending = pendingSamples.pollFirst();
        if (pending == null) {
            return Optional.empty();
        }
        CalibrationSample labeled = pending.withOutcome(outcome);
        labeledSamples.add(labeled);
        return Optional.of(labeled);
    }

    public List<CalibrationSample> getLabeledSamples() {
        return List.copyOf(labeledSamples);
    }
}

