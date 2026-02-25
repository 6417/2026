package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveToShootpos extends Command {
    private double radius = Constants.Field.RADIUS_TO_HUB;

    private final SwerveSubsystem drive;
    private final TurretSubsystem turret;

    private boolean isFinished = false;

    private Command pathCommand;
    
    public DriveToShootpos(SwerveSubsystem drive, TurretSubsystem turret) {
        this.drive = drive;
        this.turret = turret;
        addRequirements(drive);   
    }

    @Override
    public void initialize() {
        drive.setAutomatedControl();

        Pose2d nearestPos = DriverStation.getAlliance().get() == Alliance.Blue ? Constants.Field.HUB_CENTER_BLUE : Constants.Field.HUB_CENTER_RED;
        Pose2d robotPose = drive.getPose();

        // return if in neutral zone, cannot shoot from there
        if(DriverStation.getAlliance().get() == Alliance.Blue && robotPose.getX() > Constants.Field.neutralZoneStartX ||
            (DriverStation.getAlliance().get() == Alliance.Red && robotPose.getX() < Constants.Field.neutralZoneStartX)) {
            isFinished = true;
            return;
        }

        Translation2d toHub = nearestPos.getTranslation().minus(robotPose.getTranslation());

        Translation2d toPos = null;

        toPos = new Translation2d(toHub.getNorm() - radius, toHub.getAngle());

        // calculate Pose2d: nearest Spot from robot, radius(m) from Hub away
        Pose2d sweetSpot = new Pose2d(robotPose.getTranslation().plus(toPos), robotPose.getRotation().plus(RobotContainer.calculationSubsystem.getDesiredTurretAngle()));

        pathCommand = drive.driveToPose(
            sweetSpot
        );
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
        return isFinished || pathCommand.isFinished();
    }
    
}
