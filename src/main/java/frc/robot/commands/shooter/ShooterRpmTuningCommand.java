package frc.robot.commands.shooter;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/**
 * Simple shooter tuning mode:
 * - set top/bottom target RPM on dashboard
 * - command keeps applying those setpoints
 * - dashboard shows distance from current pose to the alliance hub
 */
public class ShooterRpmTuningCommand extends Command {
    private static final String TOP_RPM_KEY = "ShooterTune/SetTopRPM";
    private static final String BOTTOM_RPM_KEY = "ShooterTune/SetBottomRPM";
    private static final String DISTANCE_KEY = "ShooterTune/DistanceToHubMeters";
    private static final String ACTIVE_TOP_KEY = "ShooterTune/ActiveTopRPM";
    private static final String ACTIVE_BOTTOM_KEY = "ShooterTune/ActiveBottomRPM";

    public ShooterRpmTuningCommand() {
        addRequirements(RobotContainer.shooter);
    }

    @Override
    public void initialize() {
        SmartDashboard.setDefaultNumber(TOP_RPM_KEY, RobotContainer.shooter.getTargetTopRpm());
        SmartDashboard.setDefaultNumber(BOTTOM_RPM_KEY, RobotContainer.shooter.getTargetBottomRpm());
    }

    @Override
    public void execute() {
        RobotContainer.shooter.setTargetTopRpm(
                SmartDashboard.getNumber(TOP_RPM_KEY, RobotContainer.shooter.getTargetTopRpm()));
        RobotContainer.shooter.setTargetBottomRpm(
                SmartDashboard.getNumber(BOTTOM_RPM_KEY, RobotContainer.shooter.getTargetBottomRpm()));
        RobotContainer.shooter.applyTargetRpms();

        double distanceToHubMeters = getDistanceToHubMeters(RobotContainer.drive.getPose());
        SmartDashboard.putNumber(DISTANCE_KEY, distanceToHubMeters);
        SmartDashboard.putNumber(ACTIVE_TOP_KEY, RobotContainer.shooter.getTargetTopRpm());
        SmartDashboard.putNumber(ACTIVE_BOTTOM_KEY, RobotContainer.shooter.getTargetBottomRpm());
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private double getDistanceToHubMeters(Pose2d robotPose) {
        Pose2d hubPose = getHubPoseForCurrentAlliance();
        return robotPose.getTranslation().getDistance(hubPose.getTranslation());
    }

    private Pose2d getHubPoseForCurrentAlliance() {
        if (Constants.Field.HUB_CENTER != null) {
            return Constants.Field.HUB_CENTER;
        }

        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return Constants.Field.HUB_CENTER_RED;
        }
        return Constants.Field.HUB_CENTER_BLUE;
    }
}
