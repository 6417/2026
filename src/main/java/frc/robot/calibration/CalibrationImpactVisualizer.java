package frc.robot.calibration;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Visualizes calibration impact adjustment on a dedicated Field2d view.
 */
public class CalibrationImpactVisualizer {
    private static final String FIELD_KEY = "Calib/ImpactField2d";
    private static final int HUB_CIRCLE_POINTS = 32;

    private final Field2d field = new Field2d();
    private final FieldObject2d hubCircle = field.getObject("HubCircle");
    private final FieldObject2d shooterPoseObject = field.getObject("ShooterPose");
    private final FieldObject2d predictedImpactObject = field.getObject("PredictedImpact");
    private final FieldObject2d adjustedImpactObject = field.getObject("AdjustedImpact");
    private final FieldObject2d impactResidualObject = field.getObject("ImpactResidual");

    public CalibrationImpactVisualizer() {
        SmartDashboard.putData(FIELD_KEY, field);
    }

    public void update(
            Translation2d hubCenterField,
            double hubRadiusMeters,
            Optional<Pose2d> shooterPoseField,
            Optional<Translation2d> predictedImpactField,
            Optional<Translation2d> adjustedImpactField) {
        hubCircle.setPoses(createHubCircle(hubCenterField, hubRadiusMeters));

        if (shooterPoseField.isPresent()) {
            shooterPoseObject.setPose(shooterPoseField.get());
            field.setRobotPose(shooterPoseField.get());
        } else {
            shooterPoseObject.setPoses(List.of());
        }

        if (predictedImpactField.isPresent()) {
            predictedImpactObject.setPose(new Pose2d(predictedImpactField.get(), new Rotation2d()));
        } else {
            predictedImpactObject.setPoses(List.of());
        }

        if (adjustedImpactField.isPresent()) {
            adjustedImpactObject.setPose(new Pose2d(adjustedImpactField.get(), new Rotation2d()));
        } else {
            adjustedImpactObject.setPoses(List.of());
        }

        if (predictedImpactField.isPresent() && adjustedImpactField.isPresent()) {
            impactResidualObject.setPoses(List.of(
                    new Pose2d(predictedImpactField.get(), new Rotation2d()),
                    new Pose2d(adjustedImpactField.get(), new Rotation2d())));
        } else {
            impactResidualObject.setPoses(List.of());
        }
    }

    private List<Pose2d> createHubCircle(Translation2d center, double radiusMeters) {
        List<Pose2d> circle = new ArrayList<>(HUB_CIRCLE_POINTS + 1);
        for (int i = 0; i <= HUB_CIRCLE_POINTS; i++) {
            double angle = (2.0 * Math.PI * i) / HUB_CIRCLE_POINTS;
            Translation2d point = center.plus(new Translation2d(radiusMeters * Math.cos(angle), radiusMeters * Math.sin(angle)));
            circle.add(new Pose2d(point, new Rotation2d()));
        }
        return circle;
    }
}

