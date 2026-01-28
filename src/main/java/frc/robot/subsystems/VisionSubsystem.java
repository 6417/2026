package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.PoseEstimate;

class VisionSubsystem extends SubsystemBase {
    private String limelightName;
    private boolean mt2 = false;

    public VisionSubsystem(boolean megaTag2) {
        this.limelightName = Constants.Limelight.driveLimelight;
        this.mt2 = megaTag2;
    }

    public void changeMegaTag(boolean megaTag2) {
        this.mt2 = megaTag2;
    }

    public PoseEstimate getBotPoseEstimate_FieldSpace() {
        if (mt2) {
            return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        }
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    }

    public void updateOdometry() {
        boolean doRejectUpdate = false;

        if(mt2 == false)
        {
            LimelightHelpers.PoseEstimate mt1 = getBotPoseEstimate_FieldSpace();
            
            if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
            {
                if(mt1.rawFiducials[0].ambiguity > .7)
                {
                    doRejectUpdate = true;
                }
                if(mt1.rawFiducials[0].distToCamera > 3)
                {
                    doRejectUpdate = true;
                }
            }
            if(mt1.tagCount == 0)
            {
                doRejectUpdate = true;
            }

            if(!doRejectUpdate)
            {
                RobotContainer.drive.getSwerveDrive().addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
            }
        }
        else
        {
            LimelightHelpers.SetRobotOrientation(limelightName, RobotContainer.drive.getSwerveDrive().getYaw().getDegrees(), 0, 0, 0, 0, 0);
            LimelightHelpers.PoseEstimate mt2 = getBotPoseEstimate_FieldSpace();
            if(Math.abs(RobotContainer.drive.getSwerveDrive().getGyro().getYawAngularVelocity().abs(Units.DegreesPerSecond)) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
            {
                doRejectUpdate = true;
            }
            if(mt2.tagCount == 0)
            {
                doRejectUpdate = true;
            }
            if(!doRejectUpdate)
            {
                RobotContainer.drive.getSwerveDrive().setVisionMeasurementStdDevs(Constants.Limelight.standardDevs);
                RobotContainer.drive.getSwerveDrive().addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
            }
        }
    }
}
