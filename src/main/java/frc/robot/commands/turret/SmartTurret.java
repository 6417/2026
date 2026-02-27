package frc.robot.commands.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.Constants;
import frc.robot.utils.Utils;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;

public class SmartTurret extends Command {
    private final TurretSubsystem turret;

    public SmartTurret(TurretSubsystem turret) {
        this.turret = turret;
         
        addRequirements(turret); // don't add drive
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {
        Translation2d turretPose = drive.getPose().getTranslation().plus(Constants.TurretSubsystem.TURRET_OFFSET.rotateBy(drive.getPose().getRotation()));

        Translation2d poseToTrack = null;

        boolean toHub = false;
        // first check if is in neutral zone or team zone
        if (Utils.isRobotInNeutralZone()) {
            // in neutral zone, track edges for shooting balls in the back to team zone
            poseToTrack = Constants.Field.EDGE.getTranslation();
        } else {
            // in team zone, track hub
            poseToTrack = Constants.Field.HUB_CENTER.getTranslation();
            toHub = true;
        }
        
        // translation from robot to hub
        Translation2d turretToHub = poseToTrack.minus(turretPose);

        if (toHub) {
            // calculate distance to hub
            double distance = turretToHub.getNorm();
            turret.setDistanceHubTurret(distance);
        }

        Rotation2d angle = turretToHub.getAngle().minus(drive.getPose().getRotation());

        turret.setDesiredRotation(angle.getDegrees());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
