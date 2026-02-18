package frc.robot.calibration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.ShotKinematicSolver;

/**
 * Repräsentiert einen Feldschuss für die Kalibrierung.
 *
 * <p>Das Sample wird beim Fire erfasst (Zustand + Kommandowerte) und erst danach
 * durch den Operator mit einem {@link ShotOutcome} gelabelt.
 */
public record CalibrationSample(
        double timestampSec,
        String presetName,
        int presetStepIndex,
        String presetStepLabel,
        double presetTargetDistanceMeters,
        Pose2d robotPoseField,
        Translation2d robotVelocityFieldMps,
        double distanceToHubMeters,
        double commandedTopRpm,
        double commandedBottomRpm,
        double commandedTurretYawRad,
        ShotKinematicSolver.SolveStatus solveStatus,
        boolean rpmSaturated,
        ShotOutcome outcome,
        Translation2d predictedImpactField,
        Translation2d adjustedImpactField,
        boolean impactAdjusted,
        double impactAdjustConfidence) {

    /** Erzeugt eine Kopie desselben Samples mit neuem Label. */
    public CalibrationSample withOutcome(ShotOutcome newOutcome) {
        return new CalibrationSample(
                timestampSec,
                presetName,
                presetStepIndex,
                presetStepLabel,
                presetTargetDistanceMeters,
                robotPoseField,
                robotVelocityFieldMps,
                distanceToHubMeters,
                commandedTopRpm,
                commandedBottomRpm,
                commandedTurretYawRad,
                solveStatus,
                rpmSaturated,
                newOutcome,
                predictedImpactField,
                adjustedImpactField,
                impactAdjusted,
                impactAdjustConfidence);
    }

    /** Erzeugt eine Kopie desselben Samples mit aktualisiertem Impact-Adjustment. */
    public CalibrationSample withImpactAdjustment(Translation2d newAdjustedImpactField, double newConfidence) {
        return new CalibrationSample(
                timestampSec,
                presetName,
                presetStepIndex,
                presetStepLabel,
                presetTargetDistanceMeters,
                robotPoseField,
                robotVelocityFieldMps,
                distanceToHubMeters,
                commandedTopRpm,
                commandedBottomRpm,
                commandedTurretYawRad,
                solveStatus,
                rpmSaturated,
                outcome,
                predictedImpactField,
                newAdjustedImpactField,
                true,
                newConfidence);
    }

    /** Helper für neu erfasste Schüsse ohne manuelles Impact-Adjustment. */
    public static CalibrationSample createPending(
            double timestampSec,
            String presetName,
            int presetStepIndex,
            String presetStepLabel,
            double presetTargetDistanceMeters,
            Pose2d robotPoseField,
            Translation2d robotVelocityFieldMps,
            double distanceToHubMeters,
            double commandedTopRpm,
            double commandedBottomRpm,
            double commandedTurretYawRad,
            ShotKinematicSolver.SolveStatus solveStatus,
            boolean rpmSaturated,
            Translation2d predictedImpactField) {
        return new CalibrationSample(
                timestampSec,
                presetName,
                presetStepIndex,
                presetStepLabel,
                presetTargetDistanceMeters,
                robotPoseField,
                robotVelocityFieldMps,
                distanceToHubMeters,
                commandedTopRpm,
                commandedBottomRpm,
                commandedTurretYawRad,
                solveStatus,
                rpmSaturated,
                null,
                predictedImpactField,
                predictedImpactField,
                false,
                1.0);
    }
}

