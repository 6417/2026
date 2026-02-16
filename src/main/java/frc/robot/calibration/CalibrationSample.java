package frc.robot.calibration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.ShotKinematicSolver;

/**
 * One calibration shot sample captured at fire time and later labeled by operator.
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
        ShotOutcome outcome) {

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
                newOutcome);
    }
}

