package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.util.AbstractMap.SimpleEntry;

public class CalculationSubsystem extends SubsystemBase {
    private Rotation2d desiredTurretAngle;
    private double desiredShooterRPM;

    private double distanceHubTurret;

    public static enum ShootingMode {
        MODE_FIXED,
        MODE_STATIONARY_TURRETFIX,
        MODE_STATIONARY_TURRETDYNAMIC,
        MODE_MOVEMENT_NOROTATION,
        MODE_MOVEMENT_ROTATION
    }

    private ShootingMode currentShootingMode = ShootingMode.MODE_FIXED;

    public CalculationSubsystem() {
        Shuffleboard.getTab("Calculation").add(this);
    }

    @Override
    public void periodic() {
        switch (currentShootingMode) {
            case MODE_FIXED:
                calculateFIXED();
                break;
            case MODE_STATIONARY_TURRETFIX:
                calculateSTATIONARY_TURRETFIX();
                break;
            case MODE_STATIONARY_TURRETDYNAMIC:
                calculateSTATIONARY_TURRETDYNAMIC();
                break;
            case MODE_MOVEMENT_NOROTATION:
                calculateMOVEMENT_NOROTATION();
                break;
            case MODE_MOVEMENT_ROTATION:
                calculateMOVEMENT_ROTATION();
                break;
        }
    }

    private void calculateFIXED() {
        // fixed angle and RPM for testing
        desiredTurretAngle = Rotation2d.fromDegrees(0);
        desiredShooterRPM = Constants.Shooter.defaultRPM;
    }

    private void calculateSTATIONARY_TURRETFIX() {
        // linear interpolate distance to hub to RPM
    }

    private void calculateSTATIONARY_TURRETDYNAMIC() {
        Translation2d turretToDesiredpos = getTurretToDesiredpos();

        desiredTurretAngle = turretToDesiredpos.getAngle().minus(RobotContainer.drive.getPose().getRotation());

        // now calculate rpm
    }

    private void calculateMOVEMENT_NOROTATION() {

    }

    private void calculateMOVEMENT_ROTATION() {

    }

    private Translation2d getTurretToDesiredpos() {
        Translation2d turretPose = RobotContainer.drive.getPose().getTranslation().plus(
            Constants.TurretSubsystem.TURRET_OFFSET.rotateBy(RobotContainer.drive.getPose().getRotation()));
        Pose2d robotPose = RobotContainer.drive.getPose();

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

        // translation from turret to desired position
        Translation2d turretToDesiredpos = poseToTrack.minus(turretPose);

        if (toHub) {
            // calculate distance to hub
            distanceHubTurret = turretToDesiredpos.getNorm();
        }

        return turretToDesiredpos;
    }

    public void setShootingMode(ShootingMode mode) {
        this.currentShootingMode = mode;
    }

    public double getRPMShooter() {
        return desiredShooterRPM;
    }

    public Rotation2d getTurretAngle() {
        return desiredTurretAngle;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Desired Shooter RPM", () -> getRPMShooter(), null);
        builder.addDoubleProperty("Desired Turret Angle", () -> getTurretAngle().getDegrees(), null);
        builder.addDoubleProperty("Distance Hub Turret", () -> distanceHubTurret, null);
        super.initSendable(builder);
    }
}
