package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

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
        this.limelightOnTurretName = underTurretVision ? Constants.Limelight.underTurretLimelight : null;
        this.limelightUnderTurretName = onTurretVision ? Constants.Limelight.onTurretLimelight : null;
        this.mt2 = megaTag2;
        Constants.Limelight.useVision = this.isLimelightConnected();
    }

    public void changeMegaTag(boolean megaTag2) {
        this.mt2 = megaTag2;
    }

    @Override
    public void periodic() {
        if (this.isLimelightConnected()) {
            Constants.Limelight.useVision = true;
        } else {
            Constants.Limelight.useVision = false;
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

    public boolean isLimelightConnected() {
        return NetworkTableInstance.getDefault().getTable(limelightOnTurretName).containsKey("getpipe")
                && NetworkTableInstance.getDefault().getTable(limelightUnderTurretName).containsKey("getpipe");
    }

    public void updateOdometry() {
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
            LimelightHelpers.SetRobotOrientation(limelightOnTurretName,
                    RobotContainer.drive.getHeading().getDegrees(), 0, 0, 0, 0, 0);
            LimelightHelpers.SetRobotOrientation(limelightUnderTurretName,
                    RobotContainer.drive.getHeading().getDegrees(), 0, 0, 0, 0, 0);
            
            LimelightHelpers.PoseEstimate mt2UnderTurret = getBotPoseEstimate_fromUnderTurretLimelight_in_FieldSpace();
            LimelightHelpers.PoseEstimate mt2OnTurret = getBotPoseEstimate_fromOnTurretLimelight_in_FieldSpace();
            if (Math.abs(RobotContainer.drive.getSwerveDrive().getGyro().getYawAngularVelocity()
                    .abs(Units.DegreesPerSecond)) > 720) // if our angular velocity is greater than 720 degrees per
                                                         // second, ignore vision updates
            {
                doRejectUpdate = true;
            }
            if (mt2UnderTurret.tagCount == 0 || mt2OnTurret.tagCount == 0) {
                doRejectUpdate = true;
            }
            if (!doRejectUpdate) {
                RobotContainer.drive.getSwerveDrive()
                        .setVisionMeasurementStdDevs(Constants.Limelight.standardDevs.times(mt2UnderTurret.avgTagDist));
                RobotContainer.drive.getSwerveDrive().addVisionMeasurement(mt2UnderTurret.pose, mt2UnderTurret.timestampSeconds);
                Logger.recordOutput("UnderTurretPose", mt2UnderTurret.pose);
                Logger.recordOutput("OnTurretPose", mt2OnTurret.pose);
            }
        }
    }
}
