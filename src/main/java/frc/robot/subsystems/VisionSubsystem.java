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
    private String limelightUnderTurretName;

    public VisionSubsystem() {
        // initialise
        this.limelightUnderTurretName = Constants.Limelight.useVision
                ? Constants.Limelight.underTurretLimelight
                : null;
    }

    @Override
    public void periodic() {
        if (this.isLimelightConnected() && Constants.Limelight.useVision) {
            updateOdometryWithLimelight();
        }
        Logger.recordOutput("/Vision/LimelightConnected", this.isLimelightConnected());
        Logger.recordOutput("/Vision/UseLimelight", Constants.Limelight.useVision);

    }

    /**
     * Disable all vision fusion sources so odometry runs only from drivetrain/gyro.
     * Limelight network tables still stay alive; only the pose fusion path is disabled.
     */
    public void disableVision() {
        Constants.Limelight.useVision = false;
    }

    /**
     * Re-enable the configured limelights for odometry fusion.
     */
    public void enableVision() {
        Constants.Limelight.useVision = true;
    }

    public boolean isVisionEnabled() {
        return Constants.Limelight.useVision;
    }

    public PoseEstimate getBotPoseEstimate_from_Limelight_in_FieldSpace() {
        // Under Turret uses MT2
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightUnderTurretName);
    }

    public boolean isLimelightConnected() {
        return NetworkTableInstance.getDefault().getTable(limelightUnderTurretName).containsKey("getpipe");
    }

    /**
     * Update limelight yaw with odometry angle to prevent alliance issues when
     * initializing
     */
    public void updateLimelightYaw(String limelightName) {
        LimelightHelpers.SetRobotOrientation(limelightName, RobotContainer.drive.getHeading().getDegrees(), 0, 0, 0, 0,
                0);
    }

    private void updateOdometryWithLimelight() {
        boolean doRejectUpdate = false;
        updateLimelightYaw(limelightUnderTurretName);

        LimelightHelpers.PoseEstimate mt2 = getBotPoseEstimate_from_Limelight_in_FieldSpace();

        // Reject if spinning too fast — rolling shutter distorts tag geometry and
        // latency compensation becomes unreliable even with yaw rate provided.
        double limelightOmegaDeg = Math.abs(
                RobotContainer.gyro.getAngularVelocityZWorld().getValue().in(Units.DegreesPerSecond));
        if (limelightOmegaDeg > 45.0) {
            doRejectUpdate = true;
            Logger.recordOutput("Vision/LimelightOdometryUpdate_Rejectreason", "Spinning too fast");
        }

        // Reject if no tags visible
        if (mt2.tagCount == 0) {
            doRejectUpdate = true;
            Logger.recordOutput("Vision/LimelightOdometryUpdate_Rejectreason", "No tags visible");
        }

        // Reject if tag is too far away — pose jumps wildly at long range
        if (mt2.avgTagDist > 7.5) {
            doRejectUpdate = true;
            Logger.recordOutput("Vision/LimelightOdometryUpdate_Rejectreason", "Tag too far away");
        }

        // Reject if linear speed is too high — latency causes stale pose estimates
        edu.wpi.first.math.kinematics.ChassisSpeeds speeds = RobotContainer.drive.getRobotVelocity();
        if (Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) > 1.5) {
            doRejectUpdate = true;
            Logger.recordOutput("Vision/LimelightOdometryUpdate_Rejectreason", "Linear speed too high");
        }

        if (!doRejectUpdate) {
            // Clamp minimum distance to prevent near-zero stdDevs at close range
            double clampedDist = Math.max(mt2.avgTagDist, 0.5);
            RobotContainer.drive.getSwerveDrive()
                    .setVisionMeasurementStdDevs(Constants.Limelight.standardDevs.times(clampedDist));
            RobotContainer.drive.getSwerveDrive().addVisionMeasurement(mt2.pose,
                    mt2.timestampSeconds); // The add vision meassurement takes care of latency compensation
                                                      // internally, so we just need to pass the timestamp from the
                                                      // limelight.
            Logger.recordOutput("Vision/LimelightOdometryUpdate_Rejectreason", "None");
        }
        Logger.recordOutput("Vision/DoRejectUpdateUnderTurret", doRejectUpdate);
        Logger.recordOutput("Vision/UnderTurretPose", mt2.pose);
        Logger.recordOutput("Vision/UnderTurretTagCount", mt2.tagCount);
    }
}
