package frc.robot.subsystems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import java.util.List;

import org.ejml.dense.block.decomposition.chol.InnerCholesky_DDRB;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.LoggedTunableNumber;


public class CalculationSubsystem extends SubsystemBase {
    private Rotation2d desiredTurretAngle;
    private Pair<Double, Double> desiredShooterRPM;

    private double distanceHubTurret;
    private boolean inNeutralzone;

    // Tunable RPMs — adjustable live from the dashboard when TUNING_MODE is on.
    private final LoggedTunableNumber tuneTopRpm =
        new LoggedTunableNumber("Shooter/TuneTopRPM", Constants.Shooter.defaultRPM);
    private final LoggedTunableNumber tuneBottomRpm =
        new LoggedTunableNumber("Shooter/TuneBottomRPM", Constants.Shooter.defaultRPM);

    public static enum ShootingMode {
        MODE_FIXED,
        MODE_STATIONARY_TURRETFIX,
        MODE_STATIONARY_TURRETDYNAMIC,
        MODE_MOVEMENT_NOROTATION,
        MODE_MOVEMENT_ROTATION
    }

    private ShootingMode currentShootingMode = ShootingMode.MODE_STATIONARY_TURRETDYNAMIC;

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

        //updateDistanceToHub();
        Logger.recordOutput("Shooter/DistanceToHubMeters", distanceHubTurret);
        Logger.recordOutput("Shooter/DesiredRPMBottom", desiredShooterRPM.getFirst());
        Logger.recordOutput("Shooter/DesiredRPMTop", desiredShooterRPM.getSecond());
    }

    private void calculateFIXED() {
        // fixed angle and RPM for testing
        desiredTurretAngle = Rotation2d.fromDegrees(0);
        desiredShooterRPM = Pair.of(Constants.Shooter.defaultRPM, Constants.Shooter.defaultRPM);
    }

    private void calculateSTATIONARY_TURRETFIX() {
        // linear interpolate distance to hub to RPM
        desiredShooterRPM = shootFromDistance();
    }

    private void calculateSTATIONARY_TURRETDYNAMIC() {
        Translation2d turretToDesiredpos = getTurretToDesiredpos();

        if (turretToDesiredpos.getX() == 0 && turretToDesiredpos.getY() == 0) {
            desiredTurretAngle = Rotation2d.fromDegrees(0);
        }
        else {
            desiredTurretAngle = turretToDesiredpos.getAngle().minus(RobotContainer.drive.getPose().getRotation());
        }

        // now calculate rpm
        desiredShooterRPM = shootFromDistance();
    }

    private void calculateMOVEMENT_NOROTATION() {

    }

    private void calculateMOVEMENT_ROTATION() {

    }

    private void updateDistanceToHub() {
        if (Constants.Field.HUB_CENTER == null) return;
        Translation2d turretPose = RobotContainer.drive.getPose().getTranslation().plus(
            Constants.TurretSubsystem.TURRET_OFFSET.rotateBy(RobotContainer.drive.getPose().getRotation()));
        distanceHubTurret = Constants.Field.HUB_CENTER.getTranslation().minus(turretPose).getNorm();
    }

    private Translation2d getTurretToDesiredpos() {
        Translation2d turretPose = RobotContainer.drive.getPose().getTranslation().plus(
            Constants.TurretSubsystem.TURRET_OFFSET.rotateBy(RobotContainer.drive.getPose().getRotation()));
        Pose2d robotPose = RobotContainer.drive.getPose();

        Translation2d poseToTrack = null;
        boolean toHub = false;

        if (Constants.Field.EDGERight == null || Constants.Field.EDGELeft == null || Constants.Field.HUB_CENTER == null) {
            return new Translation2d(0,0);
        }

        Alliance all = DriverStation.getAlliance().get();

        // first check if is in neutral zone or team zone
        if ((all == Alliance.Blue && robotPose.getX() < Constants.Field.neutralZoneStartX) ||
            (all == Alliance.Red && robotPose.getX() > Constants.Field.neutralZoneStartX)) {
            // in team zone, track hub
            poseToTrack = Constants.Field.HUB_CENTER.getTranslation();
            toHub = true;
            inNeutralzone = false;
        } else {
            // in neutral zone, track edges for shooting balls in the back to team zone
            if (robotPose.getY() >= Constants.Field.FIELD_WIDTH_METERS/2) {
                //blue: left    
                //red: right
                poseToTrack = all == Alliance.Blue ? Constants.Field.EDGELeft.getTranslation() : Constants.Field.EDGERight.getTranslation();
            }
            else {
                //red: left
                //blue: right
                poseToTrack = all == Alliance.Blue ? Constants.Field.EDGERight.getTranslation() : Constants.Field.EDGELeft.getTranslation();
            }
            inNeutralzone = true;
        }

        // translation from turret to desired position
        Translation2d turretToDesiredpos = poseToTrack.minus(turretPose);

        if (toHub) {
            // calculate distance to hub
            distanceHubTurret = turretToDesiredpos.getNorm();
        }

        return turretToDesiredpos;
    }
    
    /**
     * Set the speeds of the shooter motors based on distance to the target.
     *
     * The final implementation should use measured data points and curve fitting
     * (or interpolation) to map distance -> (top RPM, bottom RPM).
     */
    private Pair<Double, Double> shootFromDistance() {
        double topRpm, bottomRpm;
        Pair<Double, Double> result;
        if (Constants.TUNING_MODE) {
            // Read live from dashboard — adjust without redeploying.
            topRpm = tuneTopRpm.get();
            bottomRpm = tuneBottomRpm.get();
        } else if(inNeutralzone) {
            topRpm = Constants.Shooter.neutralZoneRPM;
            bottomRpm = Constants.Shooter.neutralZoneRPM;
        }
        else {
            topRpm = Constants.Shooter.topRpmTable.getOutput(distanceHubTurret);
            bottomRpm = Constants.Shooter.bottomRpmTable.getOutput(distanceHubTurret);
        }

        result = Pair.of(bottomRpm, topRpm);
        return result;
    }

    public void setShootingMode(ShootingMode mode) {
        this.currentShootingMode = mode;
    }

    public Pair<Double, Double> getRPMShooter() {
        return desiredShooterRPM;
    }

    public double getDistanceToHub() {
        return distanceHubTurret;
    }

    public Rotation2d getDesiredTurretAngle() {
        return desiredTurretAngle;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleArrayProperty("Desired Shooter RPM", () -> {
            double[] i = {getRPMShooter().getFirst(), getRPMShooter().getSecond()}; return i;}, null);
        //builder.addDoubleProperty("Desired Turret Angle", () -> getDesiredTurretAngle().getDegrees(), null);
        builder.addDoubleProperty("Distance Hub Turret", () -> distanceHubTurret, null);
        super.initSendable(builder);
    }
}
