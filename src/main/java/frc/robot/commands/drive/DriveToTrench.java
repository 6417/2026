package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToTrench extends Command {

    private final SwerveSubsystem drive;
    private final Pose2d blueTrenchYlow;
    private final Pose2d blueTrenchYhigh;
    private final Pose2d redTrenchYlow;
    private final Pose2d redTrenchYhigh;
    private Command pathCommand;
    private double targetXOffsetBeforeTrench;
    private double targetYOffsetBeforeTrench;

    public DriveToTrench(SwerveSubsystem drive) {
        addRequirements(drive);
        blueTrenchYlow = new Pose2d(4.625, 0.6, new Rotation2d(0));
        blueTrenchYhigh = new Pose2d(4.625, 7.4, new Rotation2d(0));
        redTrenchYlow = new Pose2d(11.925, 0.6, new Rotation2d(Math.PI));
        redTrenchYhigh = new Pose2d(11.925, 7.4, new Rotation2d(Math.PI));
        targetXOffsetBeforeTrench = 0.75;
        targetYOffsetBeforeTrench = 0.1;
        this.drive = drive;
    }

    @Override
    public void initialize() {
        drive.setAutomatedControl();
        Pose2d robotPose = drive.getPose();
        Pose2d targetTrench = getNearestTrench(robotPose);
        Pose2d targetPoseBeforeTrench = getTargetPoseBeforeTrench(robotPose, targetTrench);
        pathCommand = drive.driveToPose(targetPoseBeforeTrench);
        pathCommand.initialize();
    }

    @Override
    public void execute() {
        if (pathCommand != null) {
            pathCommand.execute();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (pathCommand != null) {
            pathCommand.end(interrupted);
        }
        drive.setOpeatorControl();
    }

    @Override
    public boolean isFinished() {
        return pathCommand != null && pathCommand.isFinished();
    }

    private Pose2d getNearestTrench(Pose2d robotPose) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Pose2d trenchA = alliance == Alliance.Red ? redTrenchYlow : blueTrenchYlow;
        Pose2d trenchB = alliance == Alliance.Red ? redTrenchYhigh : blueTrenchYhigh;

        if (robotPose.getY() < Constants.Field.FIELD_WIDTH_METERS / 2.0) {
            return trenchA;
        } else {
            return trenchB;
        }
    }

    private Pose2d getTargetPoseBeforeTrench(Pose2d robotPose, Pose2d trench) {
        Pose2d targetBeforeTrench;
        if (robotPose.getX() > trench.getX()) {
            if (trench.getY() > Constants.Field.FIELD_WIDTH_METERS / 2.0) {
                targetBeforeTrench = new Pose2d(trench.getX() + targetXOffsetBeforeTrench,
                        trench.getY() - targetYOffsetBeforeTrench, new Rotation2d(0));
            } else {
                targetBeforeTrench = new Pose2d(trench.getX() + targetXOffsetBeforeTrench,
                        trench.getY() + targetYOffsetBeforeTrench, new Rotation2d(0));
            }
        } else {
            if (trench.getY() > Constants.Field.FIELD_WIDTH_METERS / 2.0) {
                targetBeforeTrench = new Pose2d(trench.getX() - targetXOffsetBeforeTrench,
                        trench.getY() - targetYOffsetBeforeTrench, new Rotation2d(Units.degreesToRadians(180)));
            } else {
                targetBeforeTrench = new Pose2d(trench.getX() - targetXOffsetBeforeTrench,
                        trench.getY() + targetYOffsetBeforeTrench, new Rotation2d(Units.degreesToRadians(180)));
            }
        }
        return targetBeforeTrench;

    }
}
