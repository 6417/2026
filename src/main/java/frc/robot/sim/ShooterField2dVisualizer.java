package frc.robot.sim;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Renders shooter/turret simulation objects into Field2d.
 */
public class ShooterField2dVisualizer {
    private static final String FIELD_KEY = "ShooterSim/Field2d";

    private final Field2d field = new Field2d();
    private final FieldObject2d hubObject = field.getObject("Hub");
    private final FieldObject2d turretVectorObject = field.getObject("TurretVector");
    private final FieldObject2d activeBallObject = field.getObject("ActiveBall");
    private final FieldObject2d lastHitObject = field.getObject("LastHit");
    private final FieldObject2d lastMissObject = field.getObject("LastMiss");

    private final int maxTraceCount;
    private final int maxPointsPerTrace;
    private int nextTraceIndex = 0;
    private int totalRenderedTraces = 0;

    public ShooterField2dVisualizer(Translation2d hubCenterField, int maxTraceCount, int maxPointsPerTrace) {
        this.maxTraceCount = Math.max(1, maxTraceCount);
        this.maxPointsPerTrace = Math.max(2, maxPointsPerTrace);

        hubObject.setPose(new Pose2d(hubCenterField, new Rotation2d()));
        SmartDashboard.putData(FIELD_KEY, field);
    }

    public void update(
            Pose2d robotPose,
            Translation2d muzzlePositionField,
            Rotation2d turretYawField,
            Optional<ShotSample> latestActiveShotSample,
            List<CompletedShotTrace> completedShotTraces) {
        field.setRobotPose(robotPose);
        updateTurretVector(muzzlePositionField, turretYawField);
        updateActiveBall(latestActiveShotSample);
        for (CompletedShotTrace trace : completedShotTraces) {
            renderCompletedTrace(trace);
        }
        SmartDashboard.putNumber("ShooterSim/VisibleTraces", Math.min(totalRenderedTraces, maxTraceCount));
    }

    private void updateTurretVector(Translation2d muzzlePositionField, Rotation2d turretYawField) {
        Translation2d tip = muzzlePositionField.plus(
                new Translation2d(Constants.ShooterSim.turretDisplayLengthMeters, turretYawField));
        turretVectorObject.setPoses(List.of(
                new Pose2d(muzzlePositionField, new Rotation2d()),
                new Pose2d(tip, new Rotation2d())));
    }

    private void updateActiveBall(Optional<ShotSample> latestActiveShotSample) {
        if (latestActiveShotSample.isPresent()) {
            ShotSample sample = latestActiveShotSample.get();
            activeBallObject.setPose(new Pose2d(
                    sample.positionField().getX(),
                    sample.positionField().getY(),
                    new Rotation2d()));
        } else {
            activeBallObject.setPoses(List.of());
        }
    }

    private void renderCompletedTrace(CompletedShotTrace trace) {
        List<Pose2d> poses = toField2dTracePoses(trace.samples());
        if (poses.isEmpty()) {
            return;
        }

        FieldObject2d traceObject = field.getObject("ShotTrace_" + nextTraceIndex);
        traceObject.setPoses(poses);

        Pose2d endPose = poses.get(poses.size() - 1);
        if (trace.result().hit()) {
            lastHitObject.setPose(endPose);
        } else {
            lastMissObject.setPose(endPose);
        }

        nextTraceIndex = (nextTraceIndex + 1) % maxTraceCount;
        totalRenderedTraces++;
    }

    private List<Pose2d> toField2dTracePoses(List<ShotSample> samples) {
        if (samples.isEmpty()) {
            return List.of();
        }

        int step = Math.max(1, (int) Math.ceil(samples.size() / (double) maxPointsPerTrace));
        List<Pose2d> poses = new ArrayList<>();
        for (int i = 0; i < samples.size(); i += step) {
            ShotSample sample = samples.get(i);
            poses.add(new Pose2d(
                    sample.positionField().getX(),
                    sample.positionField().getY(),
                    new Rotation2d()));
        }

        ShotSample lastSample = samples.get(samples.size() - 1);
        Pose2d lastPose = new Pose2d(lastSample.positionField().getX(), lastSample.positionField().getY(), new Rotation2d());
        if (!poses.get(poses.size() - 1).equals(lastPose)) {
            poses.add(lastPose);
        }
        return poses;
    }
}
