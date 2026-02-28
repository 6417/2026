package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

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

    public TurretSubsystem() {
        turretMotor = new FridoSparkMax(Constants.TurretSubsystem.ID); // Todo: set ID
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
            .outputRange(pidValues.peakOutputReverse, pidValues.peakOutputForward, ClosedLoopSlot.kSlot0);
        motorConfig.closedLoop.iZone(Constants.TurretSubsystem.iZone, ClosedLoopSlot.kSlot0);
        motorConfig.closedLoop.iMaxAccum(Constants.TurretSubsystem.iMaxAccum, ClosedLoopSlot.kSlot0);

        FeedForwardConfig ffConfig = new FeedForwardConfig();
        ffConfig.kS(Constants.TurretSubsystem.kFeedForward.kS);
        ffConfig.kV(Constants.TurretSubsystem.kFeedForward.kV);
        ffConfig.kA(Constants.TurretSubsystem.kFeedForward.kA);
        motorConfig.closedLoop.feedForward.apply(ffConfig); // for custom feedforward values
        
        motorConfig.smartCurrentLimit(Constants.TurretSubsystem.stallCurrentLimit, Constants.TurretSubsystem.freeCurrentLimit);

        turretMotor.asSparkMax().configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        resetRotationEncoder();

        setDefaultCommand(new SmartTurret(this));

        Shuffleboard.getTab("Turret").add(this);
    }

    public static double degs = 0;

    @Override
    public void periodic(){
    }

    public void resetRotationEncoder() {
        turretMotor.setEncoderPosition(
            this.degreesToEncoderTicks(Constants.TurretSubsystem.resetEncoderPositionDegrees)
        );
    }

    public void stopRotationMotor() {
        turretMotor.stopMotor();
    }

    // get the current angle in degrees
    public double getCurrentAngle() {
        double degs = turretMotor.getEncoderTicks() / Constants.TurretSubsystem.kGearRatio; // shift to start at 0
        degs *= Constants.TurretSubsystem.kConversationRatio;
        degs *= 360;
        return degs;
    }

    public boolean isAtSetpoint() {
        return Math.abs(turretMotor.getEncoderTicks() - desiredPosition) <= Constants.TurretSubsystem.kAllowedClosedLoopError;
    }

    // set desired rotation (in degrees!)
    public void setDesiredRotation(Rotation2d rotation) {
        //TODO: Convert Degrees to encoder ticks
        double pos = rotation.getDegrees();
        pos = clamp(pos, -90, 90); // clamp the position to the limits of the turret; here in degrees
        pos = degreesToEncoderTicks(pos);
                
        desiredPosition = pos;
        
        turretMotor.asSparkMax().getClosedLoopController().setSetpoint(pos, ControlType.kPosition);
    }

    private double degreesToEncoderTicks(double degrees) {
        double ticks = degrees / 360.0;

        ticks /= Constants.TurretSubsystem.kConversationRatio;
        ticks *= Constants.TurretSubsystem.kGearRatio;
        return ticks;
    }
        
    private double clamp(double pos, double pitchmotorreverselimit, double pitchmotorforwardlimit) {
        if (pos < pitchmotorreverselimit) {
            return pitchmotorreverselimit;
        } else if (pos > pitchmotorforwardlimit) {
            return pitchmotorforwardlimit;
        }
        return pos;
    }
        
            // set percent output for manual control.
    public void setPercent(double percent) {
        turretMotor.set(percent);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Motor Encoder Turret", () -> turretMotor.getEncoderTicks(), null);
        builder.addBooleanProperty("is at desired rotation", () -> this.isAtSetpoint(), null);
        builder.addDoubleProperty("desired rotation", () -> desiredPosition, null);
        builder.addDoubleProperty("current angle", () -> getCurrentAngle(), null);
        builder.addDoubleProperty("degrees", () -> degs, (double d) -> degs = d);
        super.initSendable(builder);
    }
}
