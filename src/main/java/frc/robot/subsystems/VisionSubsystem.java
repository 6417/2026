package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

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

    private boolean mt2;
    private double lastTurretAngleDeg = 0.0;

    public VisionSubsystem(boolean megaTag2, boolean underTurretVision, boolean onTurretVision) {
        this.limelightUnderTurretName = underTurretVision ? Constants.Limelight.underTurretLimelight : null;
        this.limelightOnTurretName = onTurretVision ? Constants.Limelight.onTurretLimelight : null;
        this.mt2 = megaTag2;
        Constants.Limelight.useVisionUnderTurret = this.isUnderTurretLimelightConnected();
        Constants.Limelight.useVisionOnTurret = this.isOnTurretLimelightConnected();
    }

    public void changeMegaTag(boolean megaTag2) {
        this.mt2 = megaTag2;
    }

    @Override
    public void periodic() {
        if (this.isUnderTurretLimelightConnected() && Constants.Limelight.useVisionUnderTurret) {
            updateOdometryWithUnderTurretLimelight();
        }
        // On-turret limelight reserved for hub aiming — not fused into odometry yet.
        if (this.isOnTurretLimelightConnected() && Constants.Limelight.useVisionOnTurret) {
            resetLimelightOnTurretPose(RobotContainer.turret.getCurrentAngle());
            // updateOdometryOnTurretLimelight();
        }
    }

    public PoseEstimate getBotPoseEstimate_fromUnderTurretLimelight_in_FieldSpace() {
        if (mt2) {
            return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightUnderTurretName);
        }
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightUnderTurretName);
    }

    public PoseEstimate getBotPoseEstimate_fromOnTurretLimelight_in_FieldSpace() {
        if (mt2) {
            return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightOnTurretName);
        }
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
        Translation2d turretRotationMiddlePoseToLimelight = new Translation2d(0.089837, 0.054311)
                .rotateBy(new Rotation2d(Math.toRadians(turretAngleDegrees)));
        double desiredX = turretRotationMiddlePose.getX() + (turretRotationMiddlePoseToLimelight.getX());
        double desiredY = turretRotationMiddlePose.getY() + (turretRotationMiddlePoseToLimelight.getY());
        double desiredZ = standardLimelightPose.getZ();
        double desiredRoll = -90;
        double desiredPitch = 28.1;
        double desiredYaw = -turretAngleDegrees; // negate because of how the limelight is mounted, so positive turret
                                                 // rotation results in negative yaw rotation of the limelight
        LimelightHelpers.setCameraPose_RobotSpace(Constants.Limelight.onTurretLimelight, desiredX, desiredY, desiredZ,
                desiredRoll, desiredPitch, desiredYaw);
    }

    /**
     * Update limelight yaw with odometry angle to prevent alliance issues when initializing
     */
    private void updateLimelightYaw(String limelightName) {
        LimelightHelpers.SetRobotOrientation(limelightName, RobotContainer.drive.getHeading().getDegrees(), 0, 0, 0, 0, 0);
    }

    public void updateOdometry() {
        updateOdometryWithUnderTurretLimelight();
        
    }

    public void updateOdometryOnTurretLimelight() {
        boolean doRejectUpdate = false;
        if (mt2 == false) {
            LimelightHelpers.PoseEstimate mt1 = getBotPoseEstimate_fromOnTurretLimelight_in_FieldSpace();
            if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
                if (mt1.rawFiducials[0].ambiguity > .7) {
                    doRejectUpdate = true;
                }
                if (mt1.rawFiducials[0].distToCamera > 3) {
                    doRejectUpdate = true;
                }
            }
            if (mt1.tagCount == 0) {
                doRejectUpdate = true;
            }

            if (!doRejectUpdate) {
                RobotContainer.drive.getSwerveDrive().addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
            }
        } else {
            updateLimelightYaw(limelightOnTurretName);

            LimelightHelpers.PoseEstimate mt2OnTurret = getBotPoseEstimate_fromOnTurretLimelight_in_FieldSpace();

            // Reject if robot spinning too fast — rolling shutter distorts tag geometry.
            double onTurretOmegaDeg = Math.abs(
                    RobotContainer.gyro.getAngularVelocityZWorld().getValue().in(Units.DegreesPerSecond));
            if (onTurretOmegaDeg > 45.0) {
                doRejectUpdate = true;
            }

            // Reject if turret slewing — camera-pose latency causes XY error proportional
            // to turret angular speed. Differentiate angle across the 20 ms loop period.
            // TODO : check if angular velocity is in the right direction
            double currentTurretAngle = RobotContainer.turret.getCurrentAngle();
            double turretSlewDegPerSec = Math.abs((currentTurretAngle - lastTurretAngleDeg) / 0.02 - RobotContainer.drive.getRobotVelocity().omegaRadiansPerSecond*180/Math.PI);
            lastTurretAngleDeg = currentTurretAngle;
            if (turretSlewDegPerSec > 30.0) {
                doRejectUpdate = true;
            }

            // Reject if no tags visible.
            if (mt2OnTurret.tagCount == 0) {
                doRejectUpdate = true;
            }

            // Reject if tag too far — MegaTag2 accuracy degrades quickly at range.
            if (mt2OnTurret.avgTagDist > 6.0) {
                doRejectUpdate = true;
            }

            // Reject if moving too fast — latency causes stale pose estimates.
            edu.wpi.first.math.kinematics.ChassisSpeeds speeds = RobotContainer.drive.getRobotVelocity();
            if (Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) > 1.5) {
                doRejectUpdate = true;
            }

            if (doRejectUpdate) {
                double clampedDist = Math.max(mt2OnTurret.avgTagDist, 0.5);
                RobotContainer.drive.getSwerveDrive()
                        .setVisionMeasurementStdDevs(Constants.Limelight.onTurretStdDevs.times(clampedDist));
                RobotContainer.drive.getSwerveDrive().addVisionMeasurement(mt2OnTurret.pose,
                        mt2OnTurret.timestampSeconds);
            }
            Logger.recordOutput("Swerve/OnTurretPose", mt2OnTurret.pose);
        }
    }

    private void updateOdometryWithUnderTurretLimelight() {
        boolean doRejectUpdate = false;
        if (mt2 == false) {
            LimelightHelpers.PoseEstimate mt1 = getBotPoseEstimate_fromUnderTurretLimelight_in_FieldSpace();
            if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
                if (mt1.rawFiducials[0].ambiguity > .7) {
                    doRejectUpdate = true;
                }
                if (mt1.rawFiducials[0].distToCamera > 3) {
                    doRejectUpdate = true;
                }
            }
            if (mt1.tagCount == 0) {
                doRejectUpdate = true;
            }

            if (!doRejectUpdate) {
                RobotContainer.drive.getSwerveDrive().addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
            }
        } else {
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
            if (mt2UnderTurret.avgTagDist > 6.0) {
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
                        mt2UnderTurret.timestampSeconds);
            }
            Logger.recordOutput("Swerve/UnderTurretPose", mt2UnderTurret.pose);
        }
    }
}
