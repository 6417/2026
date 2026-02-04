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
import frc.robot.commands.SmartTurret;

public class TurretSubsystem extends SubsystemBase {
    private FridoSparkMax turretMotor;

    private SparkBaseConfig motorConfig; 
    private MAXMotionConfig smartMotionConfig;
    private final PidValues pidValues = Constants.TurretSubsystem.pidValuesRotation;

    private double desiredPosition = 0;

    private boolean inRange = false;

    public TurretSubsystem() {
        turretMotor = new FridoSparkMax(999);/// Constants.TurretSubsystem.ID); // Todo: set ID
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

        setDefaultCommand(new SmartTurret(this, RobotContainer.drive));
    }

    @Override
    public void periodic() {
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
        return turretMotor.getEncoderTicks() / Constants.TurretSubsystem.kArmGearRatio;
    }

    public boolean isAtSetpoint() {
        return Math.abs(turretMotor.getEncoderTicks() - desiredPosition) <= Constants.TurretSubsystem.kAllowedClosedLoopError;
    }

    public boolean isInRange() { // can the turret reach the desired position? -> only 180 degrees turret
        return inRange;
    }

    public void setDesiredRotation(Rotation2d pos) {
        //TODO: set rotation with motor
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Abs Encoder Turret", () -> getAbsoluteRotation() * 360, null);
        builder.addDoubleProperty("Motor Encoder Turret", () -> turretMotor.getEncoderTicks(), null);
        builder.addBooleanProperty("is at desired rotation", () -> this.isAtSetpoint(), null);
        builder.addDoubleProperty("desired rotation", () -> desiredPosition, null);
        builder.addDoubleProperty("current angle", () -> getCurrentAngle(), null);
        builder.addDoubleProperty("absolute Rotation", () -> getAbsoluteRotation(), null);
        super.initSendable(builder);
    }
}
