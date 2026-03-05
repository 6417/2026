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

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.Constants;
import frc.robot.commands.turret.SmartTurret;

public class TurretSubsystem extends SubsystemBase {
    private FridoSparkMax turretMotor;

    private SparkBaseConfig motorConfig;
    private MAXMotionConfig smartMotionConfig;
    private SlewRateLimiter angleLimiter;
    private final PidValues pidValues = Constants.TurretSubsystem.pidValuesRotation;

    private double desiredPosition = 0;

    public TurretSubsystem() {
        turretMotor = new FridoSparkMax(Constants.TurretSubsystem.ID); // Todo: set ID
        turretMotor.setIdleMode(IdleMode.kBrake);

        motorConfig = new SparkMaxConfig();
        smartMotionConfig = new MAXMotionConfig();

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

        angleLimiter = new SlewRateLimiter(18);

        resetRotationEncoder();

        setDefaultCommand(new SmartTurret(this));


        Shuffleboard.getTab("Turret").add(this);
    }

    @Override
    public void periodic(){
    }

    public void resetRotationEncoder() {
        double ticks = this.degreesToEncoderTicks(Constants.TurretSubsystem.resetEncoderPositionDegrees);
        this.angleLimiter.reset(ticks);
        this.angleLimiter.calculate(ticks);
        turretMotor.setEncoderPosition(
            ticks
        );
    }

    public void stopRotationMotor() {
        //turretMotor.stopMotor();
    }

    public double getAmperage() {
        return turretMotor.getOutputCurrent();
    }

    public boolean isZeroDetectedByCurrent() {
        return getAmperage() >= Constants.TurretSubsystem.zeroingCurrentThresholdAmps;
    }

    // get the current angle in degrees
    public double getCurrentAngle() {
        double degs = turretMotor.getEncoderTicks() / Constants.TurretSubsystem.kGearRatio; // shift to start at 0
        degs *= Constants.TurretSubsystem.kConversationRatio;
        degs *= 360;
        return degs;
    }

    public boolean isAtSetpoint() {
        return Math.abs(turretMotor.getEncoderTicks() - desiredPosition) <= Constants.TurretSubsystem.turretTollerance;
    }

    // set desired rotation (in degrees!)
    public void setDesiredRotation(Rotation2d rotation) {
        //TODO: Convert Degrees to encoder ticks
        double pos = rotation.getDegrees();
        pos = clamp(pos, -100, 100); // clamp the position to the limits of the turret; here in degrees
        pos = degreesToEncoderTicks(pos);
                
        desiredPosition = pos;

        double smoothedPos = this.angleLimiter.calculate(pos);
        
        turretMotor.asSparkMax().getClosedLoopController().setSetpoint(smoothedPos, ControlType.kPosition);
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
    public void setVoltage(double voltage) {
        turretMotor.asSparkMax().setVoltage(voltage);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Motor Encoder Turret", () -> turretMotor.getEncoderTicks(), null);
        builder.addBooleanProperty("is at desired rotation", () -> this.isAtSetpoint(), null);
        builder.addDoubleProperty("desired rotation", () -> desiredPosition, null);
        builder.addDoubleProperty("current angle", () -> getCurrentAngle(), null);
        builder.addDoubleProperty("Amps", () -> turretMotor.getOutputCurrent(), null);
        super.initSendable(builder);
    }
}
