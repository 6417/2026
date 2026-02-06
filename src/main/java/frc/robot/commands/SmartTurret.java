package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class SmartTurret extends Command {
    private final TurretSubsystem turret;
    private final SwerveSubsystem drive;

    public SmartTurret(TurretSubsystem turret, SwerveSubsystem drive) {
       
        // Command implementation goes here
        this.drive = drive;
        this.turret = turret;
         
        addRequirements(turret); // don't add drive
    }

    @Override
    public void initialize() {
        
    }
    
    @Override
    public void execute() {
        Translation2d turretPose = drive.getPose().getTranslation().plus(Constants.TurretSubsystem.TURRET_OFFSET.rotateBy(drive.getPose().getRotation()));
        Pose2d robotPose = drive.getPose();

        Translation2d poseToTrack = null;

        boolean toHub = false;
        // first check if is in neutral zone or team zone
        if ((DriverStation.getAlliance().get() == Alliance.Blue && robotPose.getX() < Constants.Field.neutralZoneStartX) ||
            (DriverStation.getAlliance().get() == Alliance.Red && robotPose.getX() > Constants.Field.neutralZoneStartX)) {
            // in team zone, track hub
            poseToTrack = Constants.Field.HUB_CENTER.getTranslation();
            toHub = true;
        } else {
            // in neutral zone, track edges for shooting balls in the back to team zone
            poseToTrack = Constants.Field.EDGE.getTranslation();
        }
        
        // translation from robot to hub
        Translation2d turretToHub = poseToTrack.minus(turretPose);

        if (toHub) {
            // calculate distance to hub
            double distance = turretToHub.getNorm();
            turret.setDistanceHubTurret(distance);
        }

        Rotation2d angle = turretToHub.getAngle().minus(drive.getPose().getRotation());

        turret.setDesiredRotation(angle);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
