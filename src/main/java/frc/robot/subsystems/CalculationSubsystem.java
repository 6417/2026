package frc.robot.subsystems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import java.awt.Robot;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.LoggedTunableNumber;
import swervelib.simulation.ironmaple.simulation.opponentsim.SmartOpponentConfig.ChassisConfig;


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
        MODE_MOVEMENT,
        MODE_MOVEMENT_VIRTUAL,
        MODE_MOVEMENT_ROTATION
    }

    private ShootingMode currentShootingMode = ShootingMode.MODE_MOVEMENT_VIRTUAL;

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
            case MODE_MOVEMENT:
                calculateMOVEMENT();
                break;
            case MODE_MOVEMENT_VIRTUAL:
                calculateMOVEMENT_VIRTUAL();
                break;
            default:
                calculateSTATIONARY_TURRETFIX();
                break;
        }

        //updateDistanceToHub();
        Logger.recordOutput("Shooter/DistanceToHubMeters", distanceHubTurret);
        Logger.recordOutput("Shooter/DesiredRPMBottom", desiredShooterRPM.getFirst());
        Logger.recordOutput("Shooter/DesiredRPMTop", desiredShooterRPM.getSecond());

        ChassisSpeeds periodicVel = RobotContainer.drive.getFieldVelocity();
        Logger.recordOutput("ShootOnMove/RobotSpeedMps",
            Math.hypot(periodicVel.vxMetersPerSecond, periodicVel.vyMetersPerSecond));
        Logger.recordOutput("ShootOnMove/SpeedOkToShoot", isSpeedOkToShoot());
    }

    private void calculateFIXED() {
        // fixed angle and RPM for testing
        desiredTurretAngle = Rotation2d.fromDegrees(0);
        desiredShooterRPM = Pair.of(Constants.Shooter.defaultRPM, Constants.Shooter.defaultRPM);
    }

    private void calculateSTATIONARY_TURRETFIX() {
        getTurretToDesiredpos();
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

    private void calculateMOVEMENT() {
        ChassisSpeeds robotSpeed = RobotContainer.drive.getFieldVelocity();
        Translation2d robotSpeedVector = new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);

        // Get information on how you would shoot stationary at the desired position
        Translation2d turretToDesiredpos = getTurretToDesiredpos();
        desiredShooterRPM = shootFromDistance();

        // Calculate desired velocity by converting RPM to approximate ball exit velocity
        double rpm_average = (desiredShooterRPM.getFirst() + desiredShooterRPM.getSecond()) / 2;
        double rpm_to_meterspersec = 1.0 / 60.0 * Math.cos(Constants.Shooter.shooterAngle / 180.0 * Math.PI) * Math.PI*Constants.Shooter.shooterWheelDiameter_meters;
        double vball = rpm_average*rpm_to_meterspersec;
        Logger.recordOutput("Calculation/Vball", vball);
        Logger.recordOutput("Calculation/rpm_average", rpm_average);
        Logger.recordOutput("Calculation/rpm_to_meterspersec", rpm_to_meterspersec);
        Logger.recordOutput("Calculation/RobotSpeed", robotSpeedVector.getNorm());
        Translation2d vdesired = turretToDesiredpos.div(turretToDesiredpos.getNorm()).times(vball);

        // Calculate desired turret shot vector
        Translation2d vturret = vdesired.minus(robotSpeedVector);

        // Simply update the angle and do not change the RPM for now, more complex math needed otherwise
        if (vturret.getX() == 0 && vturret.getY() == 0) {
            desiredTurretAngle = Rotation2d.fromDegrees(0);
        }
        else {
            desiredTurretAngle = vturret.getAngle().minus(RobotContainer.drive.getPose().getRotation());

            double rpm_conversion_factor = 1 + Constants.Shooter.rpmConversionFactorScale * (vturret.getNorm() / vdesired.getNorm()-1);

            // Instead of treating it like this we could scale the distance to target that goes into the getdesiredRPM function with this factor (or another factor)
            // This would account for spin of the ball and could be able to shoot while driving towards the target
            desiredShooterRPM = Pair.of(desiredShooterRPM.getFirst()*(rpm_conversion_factor), desiredShooterRPM.getSecond()*rpm_conversion_factor);

            Logger.recordOutput("Calculation/rpm_conversion_factor", rpm_conversion_factor);
        }
    }

    private void calculateMOVEMENT_VIRTUAL() {
        // Reuse stationary target selection (handles hub vs neutral zone edges)
        Translation2d turretToTarget = getTurretToDesiredpos();

        if (turretToTarget.getX() == 0 && turretToTarget.getY() == 0) {
            // No target info (e.g. field measurements not set) — default to something safe.
            Logger.recordOutput("ShootOnMove/Warning", "No target info; defaulting to safe values");
            desiredTurretAngle = Rotation2d.fromDegrees(0);
            desiredShooterRPM = Pair.of(Constants.Shooter.defaultRPM, Constants.Shooter.defaultRPM);
            return;
        }

        // Iteratively refine flight time: the virtual target distance differs from
        // the raw distance, so the flight time estimate must be updated to match.
        // Contraction factor ~0.34 at max speed → 3 iterations gives <2 cm residual.
        ChassisSpeeds fieldVel = RobotContainer.drive.getFieldVelocity();
        double dist = turretToTarget.getNorm();
        double flightTime = 0;
        Translation2d turretToVirtual = turretToTarget;

        for (int i = 0; i < 3; i++) {
            flightTime = Constants.Shooter.flightTimeTable.getOutput(dist);
            turretToVirtual = new Translation2d(
                turretToTarget.getX() - fieldVel.vxMetersPerSecond * flightTime,
                turretToTarget.getY() - fieldVel.vyMetersPerSecond * flightTime);
            dist = turretToVirtual.getNorm();
        }

        desiredTurretAngle = turretToVirtual.getAngle()
            .minus(RobotContainer.drive.getPose().getRotation());

        // Update distance for RPM lookup (use virtual target distance when aiming at hub)
        if (!inNeutralzone) {
            distanceHubTurret = turretToVirtual.getNorm();
        }

        desiredShooterRPM = shootFromDistance();

        // Logging
        Logger.recordOutput("ShootOnMove/FlightTimeSec", flightTime);
        Logger.recordOutput("ShootOnMove/VirtualOffsetMeters", turretToTarget.getNorm() - dist);
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

    public boolean isSpeedOkToShoot() {
        // // TODO: 

        // ChassisSpeeds fieldVel = RobotContainer.drive.getFieldVelocity();
        // double speed = Math.hypot(fieldVel.vxMetersPerSecond, fieldVel.vyMetersPerSecond);
        // return speed <= Constants.ShootOnMove.MAX_SHOOT_SPEED_MPS;
        return true;
    }

    public void setShootingMode(ShootingMode mode) {
        this.currentShootingMode = mode;
    }

    public ShootingMode getShootingMode() {
        return currentShootingMode;
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
