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

    public VisionSubsystem(boolean megaTag2, boolean underTurretVision, boolean onTurretVision) {
        this.limelightUnderTurretName = underTurretVision ? Constants.Limelight.underTurretLimelight : null;
        this.limelightOnTurretName = onTurretVision ? Constants.Limelight.onTurretLimelight : null;
        this.mt2 = megaTag2;
        Constants.Limelight.useVisionUnderTurret = this.isUnderTurretLimelightConnected();
        Constants.Limelight.useVisionOnTurret = this.isOnTurretLimelightConnected();
        if (megaTag2) {
            // Mode 2: use external yaw (SetRobotOrientation) as full override so MegaTag2
            // uses the robot's Pigeon2 heading instead of the Limelight's internal IMU.
            if (underTurretVision) {
                LimelightHelpers.SetIMUMode(Constants.Limelight.underTurretLimelight, 2);
            }
            if (onTurretVision) {
                LimelightHelpers.SetIMUMode(Constants.Limelight.onTurretLimelight, 2);
            }
        }
    }

    public void changeMegaTag(boolean megaTag2) {
        this.mt2 = megaTag2;
    }

    @Override
    public void periodic() {
        if (this.isUnderTurretLimelightConnected() && Constants.Limelight.useVisionUnderTurret) {
            updateOdometryWithUnderTurretLimelight();
            // resetLimelightOnTurretPose(RobotContainer.turret.getCurrentAngle());
        }

        if (this.isOnTurretLimelightConnected() && Constants.Limelight.useVisionOnTurret) {
            // resetLimelightOnTurretPose(RobotContainer.turret.getCurrentAngle());
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

    public void updateOdometry() {
        updateOdometryWithUnderTurretLimelight();
        // updateOdometryOnTurretLimelight();
        // boolean doRejectUpdate = false;
        // if (mt2 == false) {
        // LimelightHelpers.PoseEstimate mt1 =
        // getBotPoseEstimate_fromUnderTurretLimelight_in_FieldSpace();
        // if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
        // if (mt1.rawFiducials[0].ambiguity > .7) {
        // doRejectUpdate = true;
        // }
        // if (mt1.rawFiducials[0].distToCamera > 3) {
        // doRejectUpdate = true;
        // }
        // }
        // if (mt1.tagCount == 0) {
        // doRejectUpdate = true;
        // }

        // if (!doRejectUpdate) {
        // RobotContainer.drive.getSwerveDrive().addVisionMeasurement(mt1.pose,
        // mt1.timestampSeconds);
        // }
        // } else {
        // LimelightHelpers.SetRobotOrientation(limelightOnTurretName,
        // RobotContainer.drive.getHeading().getDegrees(), 0, 0, 0, 0, 0);
        // LimelightHelpers.SetRobotOrientation(limelightUnderTurretName,
        // RobotContainer.drive.getHeading().getDegrees(), 0, 0, 0, 0, 0);

        // LimelightHelpers.PoseEstimate mt2UnderTurret =
        // getBotPoseEstimate_fromUnderTurretLimelight_in_FieldSpace();
        // LimelightHelpers.PoseEstimate mt2OnTurret =
        // getBotPoseEstimate_fromOnTurretLimelight_in_FieldSpace();
        // if
        // (Math.abs(RobotContainer.drive.getSwerveDrive().getGyro().getYawAngularVelocity()
        // .abs(Units.DegreesPerSecond)) > 720) // if our angular velocity is greater
        // than 720 degrees per
        // // second, ignore vision updates
        // {
        // doRejectUpdate = true;
        // }
        // if (mt2UnderTurret.tagCount == 0 || mt2OnTurret.tagCount == 0) {
        // doRejectUpdate = true;
        // }
        // if (!doRejectUpdate) {
        // RobotContainer.drive.getSwerveDrive()
        // .setVisionMeasurementStdDevs(Constants.Limelight.standardDevs.times(mt2UnderTurret.avgTagDist));
        // RobotContainer.drive.getSwerveDrive().addVisionMeasurement(mt2UnderTurret.pose,
        // mt2UnderTurret.timestampSeconds);
        // }
        // Logger.recordOutput("Swerve/UnderTurretPose", mt2UnderTurret.pose);
        // Logger.recordOutput("Swerve/OnTurretPose", mt2OnTurret.pose);
        // }
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
            LimelightHelpers.SetRobotOrientation(limelightOnTurretName,
                    RobotContainer.drive.getHeading().getDegrees(), 0, 0, 0, 0, 0);

            LimelightHelpers.PoseEstimate mt2OnTurret = getBotPoseEstimate_fromOnTurretLimelight_in_FieldSpace();
            if (Math.abs(RobotContainer.drive.getSwerveDrive().getGyro().getYawAngularVelocity()
                    .abs(Units.DegreesPerSecond)) > 720) // if our angular velocity is greater than 720 degrees per
                                                         // second, ignore vision updates
            {
                doRejectUpdate = true;
            }
            if (!doRejectUpdate) {
                RobotContainer.drive.getSwerveDrive()
                        .setVisionMeasurementStdDevs(Constants.Limelight.standardDevs.times(mt2OnTurret.avgTagDist));
                RobotContainer.drive.getSwerveDrive().addVisionMeasurement(mt2OnTurret.pose,
                        mt2OnTurret.timestampSeconds);
            }
            Logger.recordOutput("Swerve/OnTurretPose", mt2OnTurret.pose);
        }
    }

    private void updateOdometryWithUnderTurretLimelight() {
        // System.out.println("alo");
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
            LimelightHelpers.SetRobotOrientation(limelightUnderTurretName,
                    RobotContainer.drive.getHeading().getDegrees(), 0, 0, 0, 0, 0);

            LimelightHelpers.PoseEstimate mt2UnderTurret = getBotPoseEstimate_fromUnderTurretLimelight_in_FieldSpace();
            if (Math.abs(RobotContainer.drive.getSwerveDrive().getGyro().getYawAngularVelocity()
                    .abs(Units.DegreesPerSecond)) > 720) // if our angular velocity is greater than 720 degrees per
                                                         // second, ignore vision updates
            {
                doRejectUpdate = true;
            }
            if (mt2UnderTurret.tagCount == 0) {
                doRejectUpdate = true;
            }
            if (!doRejectUpdate) {
                RobotContainer.drive.getSwerveDrive()
                        .setVisionMeasurementStdDevs(Constants.Limelight.standardDevs.times(mt2UnderTurret.avgTagDist));
                RobotContainer.drive.getSwerveDrive().addVisionMeasurement(mt2UnderTurret.pose,
                        mt2UnderTurret.timestampSeconds);
            }
            Logger.recordOutput("Swerve/UnderTurretPose", mt2UnderTurret.pose);
        }
    }
}
