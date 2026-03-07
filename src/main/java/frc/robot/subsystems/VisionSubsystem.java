package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.PoseEstimate;

public class VisionSubsystem extends SubsystemBase {
    private String limelightOnTurretName;
    private String limelightUnderTurretName;

    public VisionSubsystem() {
        // initialise
        this.limelightUnderTurretName = Constants.Limelight.useVisionUnderTurret
                ? Constants.Limelight.underTurretLimelight
                : null;
        this.limelightOnTurretName = Constants.Limelight.useVisionOnTurret ? Constants.Limelight.onTurretLimelight
                : null;
        Constants.Limelight.useVisionUnderTurret = this.isUnderTurretLimelightConnected();
        Constants.Limelight.useVisionOnTurret = this.isOnTurretLimelightConnected();
    }

    @Override
    public void periodic() {
        if (this.isUnderTurretLimelightConnected() && Constants.Limelight.useVisionUnderTurret) {
            updateOdometryWithUnderTurretLimelight();
        }
        // On-turret limelight reserved for hub aiming — not fused into odometry yet.
        if (this.isOnTurretLimelightConnected() && Constants.Limelight.useVisionOnTurret) {
            resetLimelightOnTurretPose(RobotContainer.turret.getCurrentAngle());
            //updateOdometryWithOnTurretLimelight();
        }
    }

    public PoseEstimate getBotPoseEstimate_fromUnderTurretLimelight_in_FieldSpace() {
        // Under Turret uses MT2
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightUnderTurretName);
    }

    public PoseEstimate getBotPoseEstimate_fromOnTurretLimelight_in_FieldSpace() {
        // On Turret uses MT1
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightOnTurretName);
    }

    public boolean isOnTurretLimelightConnected() {
        return NetworkTableInstance.getDefault().getTable(limelightOnTurretName).containsKey("getpipe");
    }

    public boolean isUnderTurretLimelightConnected() {
        return NetworkTableInstance.getDefault().getTable(limelightUnderTurretName).containsKey("getpipe");
    }

    /** @param turretAngleDegrees Angle in degrees */
    public void resetLimelightOnTurretPose(double turretAngleDegrees) {
        Pose3d standardLimelightPose = Constants.Limelight.zeroDegreesTurretLimelightOnTurret;
        Pose3d turretRotationMiddlePose = new Pose3d(Constants.TurretSubsystem.TURRET_OFFSET.getX(),
                Constants.TurretSubsystem.TURRET_OFFSET.getY(), standardLimelightPose.getZ(), new Rotation3d());
        Translation2d turretRotationMiddlePoseToLimelight = new Translation2d(
                Constants.Limelight.turretRotationMiddlePoseToLimelight.getY(),
                Constants.Limelight.turretRotationMiddlePoseToLimelight.getX())
                .rotateBy(new Rotation2d(Math.toRadians(-turretAngleDegrees))); // ToTest: potential fix of flipping
                                                                                // turret angle :)
        double desiredX = turretRotationMiddlePose.getX() + (turretRotationMiddlePoseToLimelight.getX());
        double desiredY = turretRotationMiddlePose.getY() + (turretRotationMiddlePoseToLimelight.getY());
        double desiredZ = standardLimelightPose.getZ();
        double desiredRoll = -90;
        double desiredPitch = 28.1;
        double desiredYaw = turretAngleDegrees; // ToTest: potential fix to fix direction LL is pointing at
                                                // negate because of how the limelight is mounted, so positive turret
                                                // rotation results in negative yaw rotation of the limelight
        LimelightHelpers.setCameraPose_RobotSpace(Constants.Limelight.onTurretLimelight, desiredX, desiredY, desiredZ,
                desiredRoll, desiredPitch, desiredYaw);
    }

    /**
     * Update limelight yaw with odometry angle to prevent alliance issues when
     * initializing
     */
    private void updateLimelightYaw(String limelightName) {
        LimelightHelpers.SetRobotOrientation(limelightName, RobotContainer.drive.getHeading().getDegrees(), 0, 0, 0, 0,
                0);
    }

    public void updateOdometryWithOnTurretLimelight() {
        boolean doRejectUpdate = false;
        LimelightHelpers.PoseEstimate mt1OnTurret = getBotPoseEstimate_fromOnTurretLimelight_in_FieldSpace();

        // Reject single tag with high ambiguity
        if (mt1OnTurret.tagCount == 1 && mt1OnTurret.rawFiducials.length == 1) {
            if (mt1OnTurret.rawFiducials[0].ambiguity > .7) {
                doRejectUpdate = true;
            }
            if (mt1OnTurret.rawFiducials[0].distToCamera > 3) {
                doRejectUpdate = true;
            }
        }

        // Reject if no tags
        if (mt1OnTurret.tagCount == 0) {
            doRejectUpdate = true;
        }

        // Reject if spinning too fast (same as UnderTurret)
        double onTurretOmegaDeg = Math.abs(
                RobotContainer.gyro.getAngularVelocityZWorld().getValue().in(Units.DegreesPerSecond));
        if (onTurretOmegaDeg > 45.0) {
            doRejectUpdate = true;
        }

        if (!doRejectUpdate) {
            double clampedDist = Math.max(mt1OnTurret.avgTagDist, 0.5);
            RobotContainer.drive.getSwerveDrive()
                    .setVisionMeasurementStdDevs(Constants.Limelight.standardDevs.times(clampedDist * 2)); // On-turret limelight gets moved around more due to being on the turret, so we multiply stdDevs by 2 to account for that.
            RobotContainer.drive.getSwerveDrive().addVisionMeasurement(
                    mt1OnTurret.pose,
                    mt1OnTurret.timestampSeconds); // The add vision meassurement takes care of latency compensation internally, so we just need to pass the timestamp from the limelight.
        }

        Logger.recordOutput("Swerve/OnTurretPose", mt1OnTurret.pose);
    }

    private void updateOdometryWithUnderTurretLimelight() {
        boolean doRejectUpdate = false;
        updateLimelightYaw(limelightUnderTurretName);

        LimelightHelpers.PoseEstimate mt2UnderTurret = getBotPoseEstimate_fromUnderTurretLimelight_in_FieldSpace();

        // Reject if spinning too fast — rolling shutter distorts tag geometry and
        // latency compensation becomes unreliable even with yaw rate provided.
        double underTurretOmegaDeg = Math.abs(
                RobotContainer.gyro.getAngularVelocityZWorld().getValue().in(Units.DegreesPerSecond));
        if (underTurretOmegaDeg > 45.0) {
            doRejectUpdate = true;
        }

        // Reject if no tags visible
        if (mt2UnderTurret.tagCount == 0) {
            doRejectUpdate = true;
        }

        // Reject if tag is too far away — pose jumps wildly at long range
        if (mt2UnderTurret.avgTagDist > 7.5) {
            doRejectUpdate = true;
        }

        // Reject if linear speed is too high — latency causes stale pose estimates
        edu.wpi.first.math.kinematics.ChassisSpeeds speeds = RobotContainer.drive.getRobotVelocity();
        if (Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) > 1.5) {
            doRejectUpdate = true;
        }

        if (!doRejectUpdate) {
            // Clamp minimum distance to prevent near-zero stdDevs at close range
            double clampedDist = Math.max(mt2UnderTurret.avgTagDist, 0.5);
            RobotContainer.drive.getSwerveDrive()
                    .setVisionMeasurementStdDevs(Constants.Limelight.standardDevs.times(clampedDist));
            RobotContainer.drive.getSwerveDrive().addVisionMeasurement(mt2UnderTurret.pose,
                    mt2UnderTurret.timestampSeconds); // The add vision meassurement takes care of latency compensation internally, so we just need to pass the timestamp from the limelight.
        }
        Logger.recordOutput("Swerve/UnderTurretPose", mt2UnderTurret.pose);
    }
}
