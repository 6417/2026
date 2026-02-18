package frc.robot.calibration;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.Optional;
import java.util.UUID;

/**
 * Verwaltet den Laufzeit-Zustand einer Kalibrier-Session.
 *
 * <p>Die Session trennt sauber zwischen
 * - erfassten, aber noch nicht gelabelten Samples ({@code pendingSamples}) und
 * - final gelabelten Samples ({@code labeledSamples}).
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

    /** @return Eindeutige Session-ID für Logging und Nachvollziehbarkeit. */
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

    /** @return Optional mit aktuellem Step des Presets, sonst leer wenn Session fertig ist. */
    public Optional<CalibrationPreset.Step> getActiveStep() {
        if (activeStepIndex < 0 || activeStepIndex >= preset.steps().size()) {
            return Optional.empty();
        }
        return Optional.of(preset.steps().get(activeStepIndex));
    }

    /** @return true, wenn alle Preset-Steps abgearbeitet sind. */
    public boolean isComplete() {
        return activeStepIndex >= preset.steps().size();
    }

    /**
     * Registriert einen abgefeuerten Schuss als "pending".
     *
     * <p>Das Step-Fortschrittsmodell basiert auf der Anzahl abgefeuerter Schüsse
     * pro Step (nicht auf Labeln), damit der Ablauf für den Operator konsistent bleibt.
     */
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

    /**
     * Labelt den ältesten Pending-Schuss.
     *
     * @return Gelabeltes Sample oder leer, wenn aktuell nichts offen ist.
     */
    public Optional<CalibrationSample> recordOutcome(ShotOutcome outcome) {
        CalibrationSample pending = pendingSamples.pollFirst();
        if (pending == null) {
            return Optional.empty();
        }
        CalibrationSample labeled = pending.withOutcome(outcome);
        labeledSamples.add(labeled);
        return Optional.of(labeled);
    }

    /** @return ältestes noch ungelabeltes Sample oder leer. */
    public Optional<CalibrationSample> peekPendingSample() {
        CalibrationSample sample = pendingSamples.peekFirst();
        return sample == null ? Optional.empty() : Optional.of(sample);
    }

    /**
     * Aktualisiert das älteste Pending-Sample mit einem manuell angepassten Impactpunkt.
     *
     * @return aktualisiertes Sample oder leer, wenn aktuell kein Pending-Sample vorhanden ist.
     */
    public Optional<CalibrationSample> updatePendingImpact(edu.wpi.first.math.geometry.Translation2d adjustedImpactField,
            double confidence) {
        CalibrationSample sample = pendingSamples.pollFirst();
        if (sample == null) {
            return Optional.empty();
        }
        CalibrationSample updated = sample.withImpactAdjustment(adjustedImpactField, confidence);
        pendingSamples.addFirst(updated);
        return Optional.of(updated);
    }

    /** @return Unveränderliche Kopie aller gelabelten Samples für den Fit. */
    public List<CalibrationSample> getLabeledSamples() {
        return List.copyOf(labeledSamples);
    }
}

