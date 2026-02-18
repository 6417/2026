package frc.robot.calibration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

/**
 * Predicts a 2D impact/closest-approach point from the commanded shot state.
 */
public final class ShotImpactPredictor {
    private static final double DT_SEC = 0.005;

    public record Prediction(Translation2d impactField, boolean valid, double timeSec) {
    }

    private ShotImpactPredictor() {
    }

    public static Prediction predictFromSample(CalibrationSample sample, Translation2d hubField) {
        if (sample.solveStatus() != frc.robot.utils.ShotKinematicSolver.SolveStatus.SOLVED) {
            return new Prediction(sample.robotPoseField().getTranslation(), false, 0.0);
        }

        Pose2d robotPose = sample.robotPoseField();
        Translation2d shooterField = robotPose.getTranslation()
                .plus(Constants.Shooter.shooterOffsetRobot.rotateBy(robotPose.getRotation()));
        double avgRpm = (sample.commandedTopRpm() + sample.commandedBottomRpm()) * 0.5;
        double muzzleSpeedMps = Math.max(0.0, avgRpm * Constants.Shooter.rpmToMpsFactor);

        double yawFieldRad = robotPose.getRotation().getRadians() + sample.commandedTurretYawRad();
        double pitchRad = Constants.ShooterSim.launchPitchRad;
        double horizontalSpeed = muzzleSpeedMps * Math.cos(pitchRad);
        double vx = sample.robotVelocityFieldMps().getX() + Math.cos(yawFieldRad) * horizontalSpeed;
        double vy = sample.robotVelocityFieldMps().getY() + Math.sin(yawFieldRad) * horizontalSpeed;
        double vz = muzzleSpeedMps * Math.sin(pitchRad);

        double x = shooterField.getX();
        double y = shooterField.getY();
        double z = Constants.ShooterSim.launchHeightMeters;
        double t = 0.0;

        double hubHeight = Constants.ShooterSim.hubCenterHeightMeters;
        double bestHeightErr = Double.POSITIVE_INFINITY;
        Translation2d bestAtHubHeight = shooterField;
        double bestAtHubHeightTime = 0.0;

        boolean hadAboveHubHeight = z >= hubHeight;
        Translation2d groundImpact = shooterField;
        double groundImpactTime = 0.0;
        boolean reachedGround = false;

        while (t < Constants.ShooterSim.maxFlightTimeSec && z >= 0.0) {
            double speed = Math.sqrt(vx * vx + vy * vy + vz * vz);
            double dragFactor = Constants.ShooterSim.dragCoefficientPerMeter * speed;
            double ax = -dragFactor * vx;
            double ay = -dragFactor * vy;
            double az = -dragFactor * vz - Constants.ShooterSim.gravityMetersPerSec2;

            vx += ax * DT_SEC;
            vy += ay * DT_SEC;
            vz += az * DT_SEC;
            x += vx * DT_SEC;
            y += vy * DT_SEC;
            z += vz * DT_SEC;
            t += DT_SEC;

            double heightErr = Math.abs(z - hubHeight);
            if (heightErr < bestHeightErr) {
                bestHeightErr = heightErr;
                bestAtHubHeight = new Translation2d(x, y);
                bestAtHubHeightTime = t;
            }
            hadAboveHubHeight |= z >= hubHeight;
            if (z < 0.0) {
                groundImpact = new Translation2d(x, y);
                groundImpactTime = t;
                reachedGround = true;
                break;
            }
        }

        if (hadAboveHubHeight) {
            return new Prediction(bestAtHubHeight, true, bestAtHubHeightTime);
        }
        if (reachedGround) {
            return new Prediction(groundImpact, true, groundImpactTime);
        }
        return new Prediction(bestAtHubHeight, false, bestAtHubHeightTime);
    }
}

