package frc.robot.sim;

import java.util.List;
import java.util.Optional;
import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

/**
 * Combined mechanism + shot simulation model for shooter and turret.
 */
public class ShooterTurretSimulator {
    private final FlywheelSim topWheelSim;
    private final FlywheelSim bottomWheelSim;
    private final BallisticShotSimulator ballisticShotSimulator;

    private final double turretKp;
    private final double turretMaxVelocityRadPerSec;
    private final double launchHeightMeters;
    private final double launchPitchRad;

    private double topWheelRpm = 0.0;
    private double bottomWheelRpm = 0.0;
    private double turretAngleRad = 0.0;

    private int resolvedShotCount = 0;
    private int hitShotCount = 0;
    private Optional<ShotResult> latestShotResult = Optional.empty();
    private Optional<ShotSample> latestActiveShotSample = Optional.empty();
    private final List<CompletedShotTrace> pendingCompletedShotTraces = new ArrayList<>();

    public ShooterTurretSimulator(
            double flywheelGearing,
            double topWheelMoiKgM2,
            double bottomWheelMoiKgM2,
            double turretKp,
            double turretMaxVelocityRadPerSec,
            double launchHeightMeters,
            double launchPitchRad,
            Translation2d hubCenterField,
            double hubCenterHeightMeters,
            double hubRadiusMeters,
            double hubHeightToleranceMeters,
            double gravityMetersPerSec2,
            double maxFlightTimeSec) {
        DCMotor neoMotor = DCMotor.getNEO(1);
        this.topWheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(neoMotor, topWheelMoiKgM2, flywheelGearing),
                neoMotor);
        this.bottomWheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(neoMotor, bottomWheelMoiKgM2, flywheelGearing),
                neoMotor);
        this.turretKp = turretKp;
        this.turretMaxVelocityRadPerSec = turretMaxVelocityRadPerSec;
        this.launchHeightMeters = launchHeightMeters;
        this.launchPitchRad = launchPitchRad;
        this.ballisticShotSimulator = new BallisticShotSimulator(
                hubCenterField,
                hubCenterHeightMeters,
                hubRadiusMeters,
                hubHeightToleranceMeters,
                gravityMetersPerSec2,
                maxFlightTimeSec);
    }

    public void update(
            double dtSec,
            double batteryVolts,
            double topPercentOutput,
            double bottomPercentOutput,
            double turretTargetAngleRad) {
        topWheelSim.setInputVoltage(MathUtil.clamp(topPercentOutput, -1.0, 1.0) * batteryVolts);
        bottomWheelSim.setInputVoltage(MathUtil.clamp(bottomPercentOutput, -1.0, 1.0) * batteryVolts);
        topWheelSim.update(dtSec);
        bottomWheelSim.update(dtSec);
        topWheelRpm = topWheelSim.getAngularVelocityRPM();
        bottomWheelRpm = bottomWheelSim.getAngularVelocityRPM();

        double angleError = MathUtil.angleModulus(turretTargetAngleRad - turretAngleRad);
        double turretVelocityCmd = MathUtil.clamp(
                turretKp * angleError,
                -turretMaxVelocityRadPerSec,
                turretMaxVelocityRadPerSec);
        turretAngleRad = MathUtil.angleModulus(turretAngleRad + turretVelocityCmd * dtSec);

        ballisticShotSimulator.update(dtSec);
        latestActiveShotSample = ballisticShotSimulator.getLatestActiveShotSample();
        List<ShotResult> completed = ballisticShotSimulator.drainCompletedShots();
        pendingCompletedShotTraces.addAll(ballisticShotSimulator.drainCompletedShotTraces());
        for (ShotResult shotResult : completed) {
            latestShotResult = Optional.of(shotResult);
            resolvedShotCount++;
            if (shotResult.hit()) {
                hitShotCount++;
            }
        }
    }

    public void fireShot(
            double timestampSec,
            Pose2d robotPoseField,
            Translation2d robotVelocityFieldMps,
            Translation2d shooterOffsetRobot,
            double rpmToMpsFactor) {
        double averageWheelRpm = (topWheelRpm + bottomWheelRpm) / 2.0;
        double muzzleSpeedMps = Math.max(0.0, averageWheelRpm * rpmToMpsFactor);
        fireShotWithMuzzleSpeed(
                timestampSec,
                robotPoseField,
                robotVelocityFieldMps,
                shooterOffsetRobot,
                muzzleSpeedMps);
    }

    public void fireShotWithMuzzleSpeed(
            double timestampSec,
            Pose2d robotPoseField,
            Translation2d robotVelocityFieldMps,
            Translation2d shooterOffsetRobot,
            double muzzleSpeedMps) {
        double horizontalSpeedMps = muzzleSpeedMps * Math.cos(launchPitchRad);
        double verticalSpeedMps = muzzleSpeedMps * Math.sin(launchPitchRad);

        double yawFieldRad = robotPoseField.getRotation().getRadians() + turretAngleRad;
        Translation2d muzzleVelocityField = new Translation2d(horizontalSpeedMps, new Rotation2d(yawFieldRad));
        Translation2d initialVelocityField = robotVelocityFieldMps.plus(muzzleVelocityField);

        Translation2d shooterPositionField =
                robotPoseField.getTranslation().plus(shooterOffsetRobot.rotateBy(robotPoseField.getRotation()));

        ShotEvent shotEvent = new ShotEvent(
                timestampSec,
                new Translation3d(shooterPositionField.getX(), shooterPositionField.getY(), launchHeightMeters),
                new Translation3d(initialVelocityField.getX(), initialVelocityField.getY(), verticalSpeedMps),
                turretAngleRad,
                topWheelRpm,
                bottomWheelRpm);
        ballisticShotSimulator.queueShot(shotEvent);
    }

    public double getTopWheelRpm() {
        return topWheelRpm;
    }

    public double getBottomWheelRpm() {
        return bottomWheelRpm;
    }

    public double getTurretAngleRad() {
        return turretAngleRad;
    }

    public Optional<ShotResult> getLatestShotResult() {
        return latestShotResult;
    }

    public Optional<ShotSample> getLatestActiveShotSample() {
        return latestActiveShotSample;
    }

    public List<CompletedShotTrace> drainCompletedShotTraces() {
        List<CompletedShotTrace> drained = new ArrayList<>(pendingCompletedShotTraces);
        pendingCompletedShotTraces.clear();
        return drained;
    }

    public double getHitRate() {
        if (resolvedShotCount <= 0) {
            return 0.0;
        }
        return (double) hitShotCount / (double) resolvedShotCount;
    }
}
