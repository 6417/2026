package frc.robot.sim;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * CLI runner for scenario sweep.
 *
 * <p>Examples:
 * - Default suite: gradlew shotScenarioSweep
 * - Single scenario: gradlew shotScenarioSweep -PshotArgs="x,y,headingDeg,vx,vy"
 */
public class ShotScenarioSweepRunner {
    public static void main(String[] args) {
        ShotScenarioEvaluator evaluator = new ShotScenarioEvaluator();
        // With five args we run one explicit scenario, otherwise a compact default suite.
        List<ShotScenarioEvaluator.Scenario> scenarios = args.length == 5
                ? List.of(parseScenario(args))
                : defaultScenarios();

        int hits = 0;
        for (ShotScenarioEvaluator.Scenario scenario : scenarios) {
            ShotScenarioEvaluator.Evaluation result = evaluator.evaluate(scenario);
            if (result.hit()) {
                hits++;
            }

            System.out.printf(
                    "%s | hit=%s | turret=%.1f deg | top=%.0f rpm | bottom=%.0f rpm | muzzle=%.2f m/s | closest=%.3f m | hErr=%.3f m | vErr=%.3f m | t=%.3f s%n",
                    scenario.name(),
                    result.hit(),
                    result.turretYawRobotDeg(),
                    result.topRpm(),
                    result.bottomRpm(),
                    result.muzzleSpeedMps(),
                    result.closestDistanceMeters(),
                    result.horizontalErrorMeters(),
                    result.verticalErrorMeters(),
                    result.flightTimeSec());
        }

        System.out.printf("Summary: %d/%d hits%n", hits, scenarios.size());
    }

    private static ShotScenarioEvaluator.Scenario parseScenario(String[] args) {
        double x = Double.parseDouble(args[0]);
        double y = Double.parseDouble(args[1]);
        double headingDeg = Double.parseDouble(args[2]);
        double vx = Double.parseDouble(args[3]);
        double vy = Double.parseDouble(args[4]);

        return new ShotScenarioEvaluator.Scenario(
                "single",
                new Pose2d(x, y, Rotation2d.fromDegrees(headingDeg)),
                new Translation2d(vx, vy));
    }

    private static List<ShotScenarioEvaluator.Scenario> defaultScenarios() {
        return List.of(
                new ShotScenarioEvaluator.Scenario(
                        "center_still",
                        new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(0.0)),
                        new Translation2d(0.0, 0.0)),
                new ShotScenarioEvaluator.Scenario(
                        "left_to_right",
                        new Pose2d(2.8, 2.6, Rotation2d.fromDegrees(25.0)),
                        new Translation2d(1.2, 0.4)),
                new ShotScenarioEvaluator.Scenario(
                        "right_to_left",
                        new Pose2d(6.0, 5.5, Rotation2d.fromDegrees(-155.0)),
                        new Translation2d(-1.0, -0.5)),
                new ShotScenarioEvaluator.Scenario(
                        "fast_forward",
                        new Pose2d(4.2, 2.0, Rotation2d.fromDegrees(90.0)),
                        new Translation2d(2.0, 0.0)));
    }
}
