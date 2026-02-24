package frc.robot.subsystems;

import org.opencv.core.Mat;

import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.commands.turret.SmartTurret;

public class TurretSubsystem extends SubsystemBase {
    private FridoSparkMax turretMotor;

    private SparkBaseConfig motorConfig;
    private MAXMotionConfig smartMotionConfig;
    private final PidValues pidValues = Constants.TurretSubsystem.pidValuesRotation;

    private double desiredPosition = 0;

    private double distanceHubTurret = 0;

    public TurretSubsystem() {/*
                               * turretMotor = new FridoSparkMax(Constants.TurretSubsystem.ID); // Todo: set
                               * ID
                               * turretMotor.setIdleMode(IdleMode.kBrake);
                               * 
                               * motorConfig = new SparkMaxConfig();
                               * smartMotionConfig = new MAXMotionConfig();
                               * 
                               * SparkMaxConfig limitConfig = new SparkMaxConfig();
                               * limitConfig.softLimit
                               * .forwardSoftLimit(Constants.TurretSubsystem.pitchMotorForwardLimit).
                               * forwardSoftLimitEnabled(true);
                               * limitConfig.softLimit
                               * .reverseSoftLimit(Constants.TurretSubsystem.pitchMotorReverseLimit).
                               * reverseSoftLimitEnabled(true);
                               * 
                               * turretMotor.asSparkMax().configure(limitConfig,
                               * ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
                               * 
                               * smartMotionConfig.allowedProfileError(Constants.TurretSubsystem.
                               * kAllowedClosedLoopError, ClosedLoopSlot.kSlot0);
                               * smartMotionConfig.maxAcceleration(Constants.TurretSubsystem.kMaxAcceleration,
                               * ClosedLoopSlot.kSlot0);
                               * smartMotionConfig.cruiseVelocity(Constants.TurretSubsystem.kMaxVelocity,
                               * ClosedLoopSlot.kSlot0);
                               * smartMotionConfig.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal,
                               * ClosedLoopSlot.kSlot0);
                               * 
                               * motorConfig.closedLoop.maxMotion.apply(smartMotionConfig);
                               * 
                               * motorConfig.closedLoop.p(pidValues.kP, ClosedLoopSlot.kSlot0).i(pidValues.kI,
                               * ClosedLoopSlot.kSlot0)
                               * .d(pidValues.kD, ClosedLoopSlot.kSlot0)
                               * .outputRange(pidValues.peakOutputReverse, pidValues.peakOutputForward,
                               * ClosedLoopSlot.kSlot0);
                               * 
                               * 
                               * 
                               * FeedForwardConfig ffConfig = new FeedForwardConfig();
                               * ffConfig.kS(Constants.TurretSubsystem.kFeedForward.kS);
                               * ffConfig.kV(Constants.TurretSubsystem.kFeedForward.kV);
                               * ffConfig.kA(Constants.TurretSubsystem.kFeedForward.kA);
                               * motorConfig.closedLoop.feedForward.apply(ffConfig); // for custom feedforward
                               * values
                               * 
                               * motorConfig.smartCurrentLimit(0, 30);
                               * 
                               * turretMotor.asSparkMax().configure(motorConfig,
                               * ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
                               * 
                               * resetRotationEncoder();
                               */

        setDefaultCommand(new SmartTurret(this, RobotContainer.drive));

        Shuffleboard.getTab("Turret").add(this);
    }

    @Override
    public void periodic() {
    }

    public double getDistanceHubTurret() {
        return distanceHubTurret;
    }

    public Rotation2d getRotationToHub() {
        return new Rotation2d(Math.toRadians(desiredPosition));
    }

    public void resetRotationEncoder() {
        // turretMotor.setEncoderPosition(getAbsoluteRotation() *
        // Constants.TurretSubsystem.kGearRatio);
    }

    public void stopRotationMotor() {
        // turretMotor.stopMotor();
    }

    private double getAbsoluteRotation() {
        return 0;
        /*
         * double angle = turretMotor.asSparkMax().getAbsoluteEncoder().getPosition();
         * 
         * return angle - Constants.TurretSubsystem.angularOffset;
         */
    }

    // get the current angle in degrees
    public double getCurrentAngle() {
        return 0;
        // return turretMotor.getEncoderTicks() / Constants.TurretSubsystem.kGearRatio;
    }

    public boolean isAtSetpoint() {
        return false;
        // return Math.abs(turretMotor.getEncoderTicks() - desiredPosition) <=
        // Constants.TurretSubsystem.kAllowedClosedLoopError;
    }

    public void setDistanceHubTurret(double distance) {
        this.distanceHubTurret = distance;
    }

    /** Set desired rotation (in degrees!) */
    public void setDesiredRotation(double pos) {
        // TODO: Convert Degrees to encoder ticks
        desiredPosition = pos;

        if (Constants.Limelight.useVision) {
            resetLimelightOnTurretPose(pos);
        }
        // turretMotor.asSparkMax().getClosedLoopController().setSetpoint(pos,
        // ControlType.kMAXMotionPositionControl);
    }

    // set percent output for manual control.
    public void setPercent(double percent) {
        // turretMotor.set(percent);
    }

    public void resetLimelightOnTurretPose(double turretAngleDegrees) {
        Pose3d standardLimelightPose = Constants.Limelight.zeroDegreesTurretLimelightOnTurret;
        Pose3d turretRotationMiddlePose = new Pose3d(Constants.TurretSubsystem.TURRET_OFFSET.getX(),
                Constants.TurretSubsystem.TURRET_OFFSET.getY(), standardLimelightPose.getZ(), new Rotation3d());
        Translation2d turretRotationMiddlePoseToLimelight = new Translation2d(0.089837, 0.054311)
                .rotateBy(new Rotation2d(Math.toRadians(turretAngleDegrees)));
        double desiredX = turretRotationMiddlePose.getX() + (turretRotationMiddlePoseToLimelight.getX());
        double desiredY = turretRotationMiddlePose.getY() + (turretRotationMiddlePoseToLimelight.getY());
        double desiredZ = standardLimelightPose.getZ();
        double desiredRoll = -90;
        double desiredPitch = 28.1;
        double desiredYaw = -turretAngleDegrees; // negate because of how the limelight is mounted, so positive turret
                                                 // rotation results in negative yaw rotation of the limelight
        LimelightHelpers.setCameraPose_RobotSpace(Constants.Limelight.onTurretLimelight, desiredX, desiredY, desiredZ,
                desiredRoll, desiredPitch, desiredYaw);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Abs Encoder Turret", () -> getAbsoluteRotation() * 360, null);
        // builder.addDoubleProperty("Motor Encoder Turret", () ->
        // turretMotor.getEncoderTicks(), null);
        builder.addBooleanProperty("is at desired rotation", () -> this.isAtSetpoint(), null);
        builder.addDoubleProperty("desired rotation", () -> desiredPosition, null);
        builder.addDoubleProperty("current angle", () -> getCurrentAngle(), null);
        builder.addDoubleProperty("absolute Rotation", () -> getAbsoluteRotation(), null);
        builder.addDoubleProperty("distance turret desiredPos", () -> distanceHubTurret, null);
        super.initSendable(builder);
    }
}
