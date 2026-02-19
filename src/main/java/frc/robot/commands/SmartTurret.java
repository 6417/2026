package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
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
        Pose2d robotPose = drive.getPose();
        Translation2d turretPose = robotPose.getTranslation().plus(
                Constants.TurretSubsystem.TURRET_OFFSET.rotateBy(robotPose.getRotation()));
        Translation2d targetPosition = RobotContainer.shooter.getTargetPointForPose(robotPose);
        Translation2d turretToTarget = targetPosition.minus(turretPose);
        turret.setDistanceHubTurret(turretToTarget.getNorm());

        if (Constants.Shooter.aimAtHubOnly) {
            ChassisSpeeds fieldVelocity = drive.getFieldVelocity();
            var shotCommand = RobotContainer.shooter.calculateShotCommand(
                    robotPose,
                    new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond));
            turret.setDesiredRotation(Math.toDegrees(shotCommand.turretYawRad()));
        } else {
            Rotation2d angle = turretToTarget.getAngle().minus(robotPose.getRotation());
            turret.setDesiredRotation(angle.getDegrees());
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
