package frc.robot.calibration;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Runs operator-driven shooter calibration and applies runtime overrides.
 */
public class ShooterRealityCalibrator {
    private final ShooterSubsystem shooter;
    private final TurretSubsystem turret;
    private final SwerveSubsystem drive;

    private final List<CalibrationPreset> presets = new ArrayList<>(CalibrationPreset.defaultPresets());
    private int presetIndex = 0;

    private boolean modeEnabled = false;
    private CalibrationSession session;
    private ShooterCalibrationConfig pendingSuggestion;
    private ShooterCalibrationConfig loadedConfig;

    private final CalibrationImpactVisualizer impactVisualizer;
    private boolean impactAdjustActive = false;
    private Translation2d impactCursorField = new Translation2d();
    private Translation2d predictedImpactField = null;
    private Pose2d shooterPoseForImpact = new Pose2d();

    private String statusText = "Idle";
    private String lastCommitStatus = "none";
    private double lastDistanceErrorMeters = 0.0;
    private boolean lastDistanceWithinTolerance = true;
    private CalibrationFitter.FitResult lastFitResult = null;

    public ShooterRealityCalibrator(ShooterSubsystem shooter, TurretSubsystem turret, SwerveSubsystem drive) {
        this.shooter = shooter;
        this.turret = turret;
        this.drive = drive;
        this.impactVisualizer = new CalibrationImpactVisualizer();

        Optional<ShooterCalibrationConfig> loaded = CalibrationIO.loadBestAvailable();
        if (loaded.isPresent()) {
            loadedConfig = loaded.get();
            shooter.applyCalibrationOverrides(loadedConfig);
            lastCommitStatus = "loaded";
            statusText = "Loaded runtime/deploy calibration";
        }
        publishTelemetry();
    }

    public void periodic() {
        publishTelemetry();
    }

    public boolean isModeEnabled() {
        return modeEnabled;
    }

    public void toggleMode() {
        modeEnabled = !modeEnabled;
        statusText = modeEnabled ? "Calibration mode enabled" : "Calibration mode disabled";
        publishTelemetry();
    }

    public void setModeEnabled(boolean enabled) {
        modeEnabled = enabled;
        statusText = modeEnabled ? "Calibration mode enabled" : "Calibration mode disabled";
        publishTelemetry();
    }

    public String getSelectedPresetName() {
        return presets.get(presetIndex).name();
    }

    public void cyclePreset() {
        if (presets.isEmpty()) {
            return;
        }
        presetIndex = (presetIndex + 1) % presets.size();
        statusText = "Preset selected: " + getSelectedPresetName();
        publishTelemetry();
    }

    public void startSelectedPreset() {
        session = new CalibrationSession(presets.get(presetIndex));
        pendingSuggestion = null;
        lastFitResult = null;
        impactAdjustActive = false;
        predictedImpactField = null;
        statusText = "Started preset: " + getSelectedPresetName();
        publishTelemetry();
    }

    public void cancelSession() {
        session = null;
        pendingSuggestion = null;
        lastFitResult = null;
        impactAdjustActive = false;
        predictedImpactField = null;
        statusText = "Session canceled";
        publishTelemetry();
    }

    public boolean runNextShot() {
        if (!modeEnabled) {
            statusText = "Enable calibration mode first";
            publishTelemetry();
            return false;
        }
        if (session == null || session.isComplete()) {
            startSelectedPreset();
        }
        if (impactAdjustActive) {
            statusText = "Confirm or cancel impact adjust first";
            publishTelemetry();
            return false;
        }
        if (session.getPendingSamplesCount() > 0) {
            statusText = "Label pending sample first";
            publishTelemetry();
            return false;
        }

        ChassisSpeeds fieldVelocity = drive.getFieldVelocity();
        Pose2d robotPose = drive.getPose();
        Translation2d velocity = new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);

        shooter.applyShotCommandFromRobotState(robotPose, velocity);
        boolean fired = shooter.tryFire(turret.isAtSetpoint(), fieldVelocity.omegaRadiansPerSecond);
        if (!fired) {
            statusText = "Shot blocked: " + shooter.getLastShotBlockReason().name();
            publishTelemetry();
            return false;
        }

        Optional<ShooterSubsystem.ShotContext> contextOpt = shooter.getLastShotContext();
        if (contextOpt.isEmpty()) {
            statusText = "No shot context available";
            publishTelemetry();
            return false;
        }

        Optional<CalibrationPreset.Step> activeStep = session.getActiveStep();
        if (activeStep.isEmpty()) {
            statusText = "Preset complete";
            publishTelemetry();
            return false;
        }
        CalibrationPreset.Step step = activeStep.get();
        ShooterSubsystem.ShotContext context = contextOpt.get();

        CalibrationSample basePending = CalibrationSample.createPending(
                Timer.getFPGATimestamp(),
                session.getPreset().name(),
                session.getActiveStepIndex(),
                step.label(),
                step.targetDistanceMeters(),
                context.robotPoseField(),
                context.robotVelocityFieldMps(),
                context.distanceToHubMeters(),
                context.topRpm(),
                context.bottomRpm(),
                context.turretYawRad(),
                context.solveStatus(),
                context.rpmSaturated(),
                context.robotPoseField().getTranslation());

        ShotImpactPredictor.Prediction prediction = ShotImpactPredictor.predictFromSample(
                basePending,
                Constants.Shooter.hubPositionField);

        CalibrationSample pending = new CalibrationSample(
                basePending.timestampSec(),
                basePending.presetName(),
                basePending.presetStepIndex(),
                basePending.presetStepLabel(),
                basePending.presetTargetDistanceMeters(),
                basePending.robotPoseField(),
                basePending.robotVelocityFieldMps(),
                basePending.distanceToHubMeters(),
                basePending.commandedTopRpm(),
                basePending.commandedBottomRpm(),
                basePending.commandedTurretYawRad(),
                basePending.solveStatus(),
                basePending.rpmSaturated(),
                null,
                prediction.impactField(),
                prediction.impactField(),
                false,
                Constants.Calibration.impactAdjustDefaultConfidence);

        session.registerFiredShot(pending);
        pendingSuggestion = null;

        shooterPoseForImpact = new Pose2d(
                context.robotPoseField().getTranslation().plus(
                        Constants.Shooter.shooterOffsetRobot.rotateBy(context.robotPoseField().getRotation())),
                new Rotation2d());
        predictedImpactField = prediction.impactField();
        impactCursorField = prediction.impactField();
        impactAdjustActive = prediction.valid();

        lastDistanceErrorMeters = Math.abs(context.distanceToHubMeters() - step.targetDistanceMeters());
        lastDistanceWithinTolerance = lastDistanceErrorMeters <= Constants.Calibration.stepDistanceToleranceMeters;

        statusText = impactAdjustActive
                ? "Shot captured. Adjust red impact point with D-pad and confirm with A."
                : "Shot captured. Prediction invalid, label with X/B or D-pad.";
        publishTelemetry();
        return true;
    }

    public boolean isImpactAdjustActive() {
        return impactAdjustActive;
    }

    public boolean nudgeImpactCursor(double dxMeters, double dyMeters) {
        if (!impactAdjustActive) {
            return false;
        }

        double newX = impactCursorField.getX() + dxMeters;
        double newY = impactCursorField.getY() + dyMeters;
        newX = Math.max(0.0, Math.min(Constants.Field.FIELD_LENGTH_METERS, newX));
        newY = Math.max(0.0, Math.min(Constants.Field.FIELD_WIDTH_METERS, newY));
        impactCursorField = new Translation2d(newX, newY);
        statusText = String.format("Impact cursor: x=%.2f y=%.2f", impactCursorField.getX(), impactCursorField.getY());
        publishTelemetry();
        return true;
    }

    public boolean confirmImpactAdjust() {
        if (!impactAdjustActive || session == null) {
            return false;
        }
        Optional<CalibrationSample> updated = session.updatePendingImpact(
                impactCursorField,
                Constants.Calibration.impactAdjustDefaultConfidence);
        if (updated.isEmpty()) {
            statusText = "No pending sample for impact adjust";
            publishTelemetry();
            return false;
        }
        impactAdjustActive = false;
        statusText = "Impact point confirmed. Label with X/B or D-pad.";
        publishTelemetry();
        return true;
    }

    public void cancelImpactAdjust() {
        if (!impactAdjustActive) {
            return;
        }
        impactAdjustActive = false;
        statusText = "Impact adjust canceled. Label with X/B or D-pad.";
        publishTelemetry();
    }

    public boolean markHit() {
        return recordOutcome(ShotOutcome.HIT, "Marked HIT");
    }

    public boolean markMissUnknown() {
        return recordOutcome(ShotOutcome.MISS_UNKNOWN, "Marked MISS");
    }

    public boolean markMissLeft() {
        return recordOutcome(ShotOutcome.MISS_LEFT, "Marked MISS_LEFT");
    }

    public boolean markMissRight() {
        return recordOutcome(ShotOutcome.MISS_RIGHT, "Marked MISS_RIGHT");
    }

    public boolean markMissShort() {
        return recordOutcome(ShotOutcome.MISS_SHORT, "Marked MISS_SHORT");
    }

    public boolean markMissLong() {
        return recordOutcome(ShotOutcome.MISS_LONG, "Marked MISS_LONG");
    }

    public Optional<ShooterCalibrationConfig> computeSuggestion() {
        if (session == null) {
            statusText = "No active session";
            publishTelemetry();
            return Optional.empty();
        }

        List<CalibrationSample> samples = session.getLabeledSamples();
        if (samples.size() < Constants.Calibration.minSamplesForFit) {
            statusText = "Need at least " + Constants.Calibration.minSamplesForFit + " labeled shots";
            publishTelemetry();
            return Optional.empty();
        }

        double[] knotDistances = Arrays.copyOf(
                Constants.Calibration.biasFitDistancesMeters,
                Constants.Calibration.biasFitDistancesMeters.length);
        double[] baseBiasValues = new double[knotDistances.length];
        for (int i = 0; i < knotDistances.length; i++) {
            baseBiasValues[i] = shooter.getActiveDistanceScaleBias(knotDistances[i]);
        }

        CalibrationFitter.FitResult fit = CalibrationFitter.fit(
                samples,
                shooter.getActiveTurretZeroOffsetRad(),
                baseBiasValues,
                new CalibrationFitter.Params(
                        knotDistances,
                        Constants.Calibration.biasWeightWindowMeters,
                        Constants.Calibration.biasLearningRate,
                        Constants.Calibration.maxBiasRelativeStep,
                        Constants.Calibration.minBiasValue,
                        Constants.Calibration.maxBiasValue,
                        Constants.Calibration.minDirectionalSamplesForTurretFit,
                        Constants.Calibration.turretDegPerDirectionalMiss,
                        Constants.Calibration.maxTurretOffsetDeg,
                        Constants.Calibration.impactResidualWeight,
                        Constants.Calibration.impactRangeMetersPerBiasUnit,
                        Constants.Calibration.impactLateralMetersPerTurretDeg,
                        Constants.Calibration.turretResidualLearningRate,
                        Constants.Calibration.biasResidualLearningRate,
                        Constants.Calibration.maxTurretDeltaDegPerRound,
                        Constants.Calibration.maxBiasDeltaPerRound,
                        Constants.Calibration.minLateralWeightForTurretUpdate,
                        Constants.Calibration.minRangeWeightForBiasUpdate,
                        Constants.Calibration.fitShrinkFactorLowConfidence,
                        Constants.Calibration.residualHuberDeltaMeters));

        ShooterCalibrationConfig suggestion = new ShooterCalibrationConfig();
        suggestion.version = 1;
        suggestion.source = "ShooterRealityCalibrator";
        suggestion.sampleCount = samples.size();
        suggestion.turretZeroOffsetRad = fit.turretZeroOffsetRad();
        suggestion.biasDistancesM = knotDistances;
        suggestion.biasValues = fit.biasValues();

        lastFitResult = fit;
        pendingSuggestion = suggestion;
        statusText = "Suggestion ready (" + samples.size() + " samples)";
        publishTelemetry();
        return Optional.of(suggestion);
    }

    public boolean commitSuggestion() {
        if (pendingSuggestion == null) {
            statusText = "No suggestion to commit";
            publishTelemetry();
            return false;
        }

        shooter.applyCalibrationOverrides(pendingSuggestion);
        boolean saved = CalibrationIO.writeRuntimeConfig(pendingSuggestion);
        if (saved) {
            loadedConfig = pendingSuggestion.copy();
            lastCommitStatus = "saved";
            statusText = "Calibration committed and saved";
        } else {
            lastCommitStatus = "save_failed";
            statusText = "Applied in RAM, save failed";
        }
        publishTelemetry();
        return true;
    }

    private boolean recordOutcome(ShotOutcome outcome, String successMessage) {
        if (impactAdjustActive) {
            statusText = "Confirm/cancel impact adjust first";
            publishTelemetry();
            return false;
        }
        if (session == null) {
            statusText = "No active session";
            publishTelemetry();
            return false;
        }
        Optional<CalibrationSample> labeled = session.recordOutcome(outcome);
        if (labeled.isEmpty()) {
            statusText = "No pending shot to label";
            publishTelemetry();
            return false;
        }
        pendingSuggestion = null;
        lastFitResult = null;
        statusText = successMessage;
        publishTelemetry();
        return true;
    }

    private void publishTelemetry() {
        SmartDashboard.putBoolean("Calib/ModeActive", modeEnabled);
        SmartDashboard.putString("Calib/Status", statusText);
        SmartDashboard.putString("Calib/Preset", getSelectedPresetName());
        SmartDashboard.putBoolean("Calib/SessionActive", session != null);
        SmartDashboard.putString("Calib/SessionId", session == null ? "" : session.getSessionId());
        SmartDashboard.putNumber("Calib/StepIndex", session == null ? -1 : session.getActiveStepIndex());
        SmartDashboard.putNumber("Calib/ShotsInStep", session == null ? 0 : session.getShotsFiredInStep());
        SmartDashboard.putNumber("Calib/PendingLabels", session == null ? 0 : session.getPendingSamplesCount());
        SmartDashboard.putNumber("Calib/LabeledSamples", session == null ? 0 : session.getLabeledSampleCount());
        SmartDashboard.putBoolean("Calib/SessionComplete", session != null && session.isComplete());
        if (session != null && session.getActiveStep().isPresent()) {
            CalibrationPreset.Step step = session.getActiveStep().get();
            SmartDashboard.putString("Calib/StepLabel", step.label());
            SmartDashboard.putNumber("Calib/StepTargetDistanceM", step.targetDistanceMeters());
            SmartDashboard.putNumber("Calib/StepTargetVx", step.targetRobotVelocityFieldMps().getX());
            SmartDashboard.putNumber("Calib/StepTargetVy", step.targetRobotVelocityFieldMps().getY());
        } else {
            SmartDashboard.putString("Calib/StepLabel", "n/a");
            SmartDashboard.putNumber("Calib/StepTargetDistanceM", 0.0);
            SmartDashboard.putNumber("Calib/StepTargetVx", 0.0);
            SmartDashboard.putNumber("Calib/StepTargetVy", 0.0);
        }

        SmartDashboard.putBoolean("Calib/SuggestionReady", pendingSuggestion != null);
        if (pendingSuggestion != null) {
            SmartDashboard.putNumber("Calib/SuggestedTurretOffsetDeg",
                    Math.toDegrees(pendingSuggestion.turretZeroOffsetRad));
        } else {
            SmartDashboard.putNumber("Calib/SuggestedTurretOffsetDeg", 0.0);
        }

        SmartDashboard.putString("Calib/CommitStatus", lastCommitStatus);
        SmartDashboard.putBoolean("Calib/LoadedConfigPresent", loadedConfig != null);
        SmartDashboard.putNumber("Calib/LastDistanceErrorM", lastDistanceErrorMeters);
        SmartDashboard.putBoolean("Calib/LastDistanceInTolerance", lastDistanceWithinTolerance);
        SmartDashboard.putNumber("Calib/FitDirectionalSamples", lastFitResult == null ? 0 : lastFitResult.directionalSamples());
        SmartDashboard.putNumber("Calib/FitShortLongSamples", lastFitResult == null ? 0 : lastFitResult.shortLongSamples());
        SmartDashboard.putBoolean("Calib/FitTurretSuppressed", lastFitResult != null && lastFitResult.turretUpdateSuppressed());
        SmartDashboard.putNumber("Calib/FitTurretDeltaDeg",
                lastFitResult == null ? 0.0 : Math.toDegrees(lastFitResult.turretDeltaRad()));
        SmartDashboard.putNumber("Calib/FitLateralWeight",
                lastFitResult == null ? 0.0 : lastFitResult.lateralWeightUsed());
        SmartDashboard.putNumber("Calib/FitSuppressedBiasKnots",
                lastFitResult == null ? 0 : lastFitResult.suppressedBiasKnots());
        for (int i = 0; i < Constants.Calibration.biasFitDistancesMeters.length; i++) {
            double rangeWeight = 0.0;
            if (lastFitResult != null && i < lastFitResult.rangeWeightPerKnot().length) {
                rangeWeight = lastFitResult.rangeWeightPerKnot()[i];
            }
            SmartDashboard.putNumber("Calib/FitRangeWeightKnot" + i, rangeWeight);
        }
        SmartDashboard.putBoolean("Calib/ImpactAdjustActive", impactAdjustActive);
        SmartDashboard.putNumber("Calib/ImpactCursorX", impactCursorField.getX());
        SmartDashboard.putNumber("Calib/ImpactCursorY", impactCursorField.getY());
        SmartDashboard.putNumber("Calib/PredictedImpactX", predictedImpactField == null ? 0.0 : predictedImpactField.getX());
        SmartDashboard.putNumber("Calib/PredictedImpactY", predictedImpactField == null ? 0.0 : predictedImpactField.getY());

        impactVisualizer.update(
                Constants.Shooter.hubPositionField,
                Constants.ShooterSim.hubRadiusMeters,
                Optional.of(shooterPoseForImpact),
                Optional.ofNullable(predictedImpactField),
                impactAdjustActive ? Optional.of(impactCursorField) : Optional.empty());
    }
}
