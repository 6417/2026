package frc.robot.subsystems;

import com.fasterxml.jackson.annotation.JsonPropertyDescription;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

public class TurretSubsystem extends SubsystemBase {
    private FridoSparkMax turretMotor;

    private SparkBaseConfig motorConfig; 
    private MAXMotionConfig smartMotionConfig;
    private final PidValues pidValues = Constants.TurretSubsystem.pidValuesRotation;

    private double desiredAngleDegrees = 0;

    private boolean inRange = false;

    private static final Pose2d HUB_CENTER = 
        DriverStation.getAlliance().get() == Alliance.Blue ? 
        Constants.Field.HUB_CENTER_BLUE : 
        Constants.Field.HUB_CENTER_RED;
    private static final Pose2d EDGE =
        DriverStation.getAlliance().get() == Alliance.Blue ? 
        new Pose2d(0, 0, null) : 
        new Pose2d(Constants.Field.FIELD_LENGTH_METERS, Constants.Field.FIELD_WIDTH_METERS, null);
    
    private double neutralZoneStartX = DriverStation.getAlliance().get() == Alliance.Blue ? Units.inchesToMeters(158.6) : Units.inchesToMeters(Constants.Field.FIELD_LENGTH_INCHES -158.6);

    public TurretSubsystem() {
        turretMotor = new FridoSparkMax(Constants.TurretSubsystem.ID);
        turretMotor.setIdleMode(IdleMode.kBrake);

        motorConfig = new SparkMaxConfig();
        smartMotionConfig = new MAXMotionConfig();

        SparkMaxConfig limitConfig = new SparkMaxConfig();
        limitConfig.softLimit
        .forwardSoftLimit(Constants.TurretSubsystem.pitchMotorForwardLimit).forwardSoftLimitEnabled(true);
        limitConfig.softLimit
        .reverseSoftLimit(Constants.TurretSubsystem.pitchMotorReverseLimit).reverseSoftLimitEnabled(true);

        turretMotor.asSparkMax().configure(limitConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        smartMotionConfig.allowedProfileError(Constants.TurretSubsystem.kAllowedClosedLoopError, ClosedLoopSlot.kSlot0);
        smartMotionConfig.maxAcceleration(Constants.TurretSubsystem.kMaxAcceleration, ClosedLoopSlot.kSlot0);
        smartMotionConfig.cruiseVelocity(Constants.TurretSubsystem.kMaxVelocity, ClosedLoopSlot.kSlot0);
        smartMotionConfig.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot0);

        motorConfig.closedLoop.maxMotion.apply(smartMotionConfig);

        motorConfig.closedLoop.p(pidValues.kP, ClosedLoopSlot.kSlot0).i(pidValues.kI, ClosedLoopSlot.kSlot0)
            .d(pidValues.kD, ClosedLoopSlot.kSlot0)
            .outputRange(pidValues.peakOutputReverse, pidValues.peakOutputForward, ClosedLoopSlot.kSlot0)
            .velocityFF(pidValues.kF.orElse(0.0), ClosedLoopSlot.kSlot0);
        
        //    motorConfig.closedLoop.feedForward.apply(); // for custom feedforward values
        
        motorConfig.smartCurrentLimit(0, 30);

        turretMotor.asSparkMax().configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        resetRotationEncoder();
    }

    @Override
    public void periodic() {
        Pose2d robotPose = RobotContainer.drive.getPose();
        
        Pose2d poseToTrack = null;
        // first check if is in neutral zone or team zone
        if ((DriverStation.getAlliance().get() == Alliance.Blue && robotPose.getX() < neutralZoneStartX) ||
            (DriverStation.getAlliance().get() == Alliance.Red && robotPose.getX() > neutralZoneStartX)) {
            // in team zone, track hub
            poseToTrack = HUB_CENTER;
        } else {
            // in neutral zone, track edges for shooting balls in the back to team zone
            poseToTrack = EDGE;
        }

        // translation from robot to hub
        Translation2d robotToHub = new Translation2d(robotPose.getX() - poseToTrack.getX(), robotPose.getY() - poseToTrack.getY());
        
        Rotation2d angleToHub = new Rotation2d(Math.atan2(robotToHub.getY(), robotToHub.getX()));
    
        Rotation2d turretAngle = angleToHub.minus(robotPose.getRotation());
       
        double turretRotations = turretAngle.getRotations();
        
        turretRotations = clampToTurretLimits(turretRotations);
        
        setDesiredRotation(turretRotations);
    }

    private double clampToTurretLimits(double rotations) {
        double minRotations = Constants.TurretSubsystem.pitchMotorReverseLimit / Constants.TurretSubsystem.kArmGearRatio;
        double maxRotations = Constants.TurretSubsystem.pitchMotorForwardLimit / Constants.TurretSubsystem.kArmGearRatio;
        return Math.max(minRotations, Math.min(maxRotations, rotations));
    }

    public void resetRotationEncoder() {
        turretMotor.setEncoderPosition(getAbsoluteRotation() * Constants.TurretSubsystem.kArmGearRatio);
    }

    public void stopRotationMotor() {
        turretMotor.stopMotor();
    }

    private double getAbsoluteRotation() {
        double angle = turretMotor.asSparkMax().getAbsoluteEncoder().getPosition();
        
        return angle - Constants.TurretSubsystem.angularOffset;
    }


    // get the current angle in degrees
    public double getCurrentAngle() {

    }

    public boolean isAtSetpoint() {
        return Math.abs(turretMotor.getEncoderTicks() - desiredAngleDegrees) <= Constants.TurretSubsystem.kAllowedClosedLoopError;
    }

    public boolean isInRange() { // can the turret reach the desired position? -> only 180 degrees turret
        return inRange;
    }

    public void setDesiredRotation(double pos) {
        desiredAngleDegrees = pos;
        double motorSetpoint = pos * Constants.TurretSubsystem.kArmGearRatio;
        turretMotor.asSparkMax().getClosedLoopController()
            .setSetpoint(motorSetpoint, ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Abs Encoder Turret", () -> getAbsoluteRotation() * 360, null);
        builder.addDoubleProperty("Motor Encoder Turret", () -> turretMotor.getEncoderTicks(), null);
        builder.addBooleanProperty("is at desired rotation", () -> this.isAtSetpoint(), null);
        builder.addDoubleProperty("desired rotation", () -> desiredAngleDegrees, null);
        builder.addDoubleArrayProperty("current angle", () -> getCurrentAngle(), null);
        super.initSendable(builder);
    }
}
