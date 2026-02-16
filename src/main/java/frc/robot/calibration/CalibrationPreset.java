package frc.robot.calibration;

import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Operator-driven test sequence used for repeatable field calibration sessions.
 */
public record CalibrationPreset(String name, String description, List<Step> steps) {
    public record Step(String label, double targetDistanceMeters, Translation2d targetRobotVelocityFieldMps, int shotsRequired) {
    }

    public static List<CalibrationPreset> defaultPresets() {
        return List.of(
                new CalibrationPreset(
                        "StaticLadder",
                        "Stationary 2m->5m distance ladder.",
                        List.of(
                                new Step("2.0m static", 2.0, new Translation2d(), 3),
                                new Step("3.0m static", 3.0, new Translation2d(), 3),
                                new Step("4.0m static", 4.0, new Translation2d(), 3),
                                new Step("5.0m static", 5.0, new Translation2d(), 3))),
                new CalibrationPreset(
                        "MovingLateral",
                        "Lateral moving shots around 3m and 4.5m.",
                        List.of(
                                new Step("3.0m +1.0mps Y", 3.0, new Translation2d(0.0, 1.0), 3),
                                new Step("3.0m -1.0mps Y", 3.0, new Translation2d(0.0, -1.0), 3),
                                new Step("4.5m +1.0mps Y", 4.5, new Translation2d(0.0, 1.0), 3),
                                new Step("4.5m -1.0mps Y", 4.5, new Translation2d(0.0, -1.0), 3))),
                new CalibrationPreset(
                        "Mixed",
                        "Mixed distance/speed sanity set.",
                        List.of(
                                new Step("2.5m static", 2.5, new Translation2d(), 2),
                                new Step("3.5m static", 3.5, new Translation2d(), 2),
                                new Step("4.5m +1.5mps X", 4.5, new Translation2d(1.5, 0.0), 2),
                                new Step("5.5m +1.5mps Y", 5.5, new Translation2d(0.0, 1.5), 2))));
    }
}

