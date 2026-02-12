package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.SmartTurret;

public class TurretSubsystem extends SubsystemBase {
    private FridoSparkMax turretMotor;

    private SparkBaseConfig motorConfig; 
    private MAXMotionConfig smartMotionConfig;
    private final PidValues pidValues = Constants.TurretSubsystem.pidValuesRotation;

    private static final double LOOP_PERIOD_SEC = 0.02;
    private static final double SIM_MAX_SPEED_DEG_PER_SEC = 360.0;
    private static final double MANUAL_PERCENT_TO_DEG_PER_SEC = 180.0;
    private static final double SETPOINT_TOLERANCE_DEG = 1.0;

    private double desiredPosition = 0;
    private double simulatedCurrentAngleDeg = 0.0;

    private double distanceHubTurret = 0;

    public TurretSubsystem() {/*
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
        


        FeedForwardConfig ffConfig = new FeedForwardConfig();
        ffConfig.kS(Constants.TurretSubsystem.kFeedForward.kS);
        ffConfig.kV(Constants.TurretSubsystem.kFeedForward.kV);
        ffConfig.kA(Constants.TurretSubsystem.kFeedForward.kA);
        motorConfig.closedLoop.feedForward.apply(ffConfig); // for custom feedforward values
        
        motorConfig.smartCurrentLimit(0, 30);

        turretMotor.asSparkMax().configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        resetRotationEncoder();*/

        setDefaultCommand(new SmartTurret(this, RobotContainer.drive));

        Shuffleboard.getTab("Turret").add(this);
    }

    @Override
    public void periodic() {
        if (turretMotor == null) {
            // Keep a simple internal turret state so SmartTurret + sim can use a realistic angle.
            double errorDeg = MathUtil.inputModulus(desiredPosition - simulatedCurrentAngleDeg, -180.0, 180.0);
            double maxStepDeg = SIM_MAX_SPEED_DEG_PER_SEC * LOOP_PERIOD_SEC;
            simulatedCurrentAngleDeg = MathUtil.inputModulus(
                    simulatedCurrentAngleDeg + MathUtil.clamp(errorDeg, -maxStepDeg, maxStepDeg),
                    -180.0,
                    180.0);
        }

        syncShooterTargetInSimulation();
    }

    public double getDistanceHubTurret() {
        return distanceHubTurret;
    }

    public void resetRotationEncoder() {
        //turretMotor.setEncoderPosition(getAbsoluteRotation() * Constants.TurretSubsystem.kGearRatio);
    }

    public void stopRotationMotor() {
        //turretMotor.stopMotor();
    }

    private double getAbsoluteRotation() {
        // Return absolute rotation in turns to stay compatible with existing dashboard property.
        return getCurrentAngle() / 360.0;
        /* 
        double angle = turretMotor.asSparkMax().getAbsoluteEncoder().getPosition();
        
        return angle - Constants.TurretSubsystem.angularOffset;*/
    }

    // get the current angle in degrees
    public double getCurrentAngle() {
        if (turretMotor != null) {
            // TODO: Replace with real encoder conversion once turret hardware config is enabled.
            // return turretMotor.getEncoderTicks() / Constants.TurretSubsystem.kGearRatio;
        }
        return simulatedCurrentAngleDeg;
        //return turretMotor.getEncoderTicks() / Constants.TurretSubsystem.kGearRatio;
    }

    public boolean isAtSetpoint() {
        double errorDeg = MathUtil.inputModulus(desiredPosition - getCurrentAngle(), -180.0, 180.0);
        return Math.abs(errorDeg) <= SETPOINT_TOLERANCE_DEG;
    }

    public void setDistanceHubTurret(double distance) {
        this.distanceHubTurret = distance;
    }

    // set desired rotation (in degrees!)
    public void setDesiredRotation(double pos) {
        desiredPosition = MathUtil.inputModulus(pos, -180.0, 180.0);

        // TODO: Convert desiredPosition to motor rotations/ticks and use closed-loop setpoint on real hardware.
        // turretMotor.asSparkMax().getClosedLoopController().setSetpoint(desiredPosition, ControlType.kMAXMotionPositionControl);

        syncShooterTargetInSimulation();
    }

    // set percent output for manual control.
    public void setPercent(double percent) {
        double clampedPercent = MathUtil.clamp(percent, -1.0, 1.0);
        if (turretMotor != null) {
            turretMotor.set(clampedPercent);
            return;
        }

        // In sim/no-hardware mode, manual control updates the target angle directly.
        desiredPosition = MathUtil.inputModulus(
                desiredPosition + clampedPercent * MANUAL_PERCENT_TO_DEG_PER_SEC * LOOP_PERIOD_SEC,
                -180.0,
                180.0);
        syncShooterTargetInSimulation();
    }

    public double getDesiredAngle() {
        return desiredPosition;
    }

    private void syncShooterTargetInSimulation() {
        if (RobotBase.isSimulation() && RobotContainer.shooter != null) {
            RobotContainer.shooter.setTurretTargetAngleRad(Math.toRadians(desiredPosition));
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Abs Encoder Turret", () -> getAbsoluteRotation() * 360, null);
        //builder.addDoubleProperty("Motor Encoder Turret", () -> turretMotor.getEncoderTicks(), null);
        builder.addBooleanProperty("is at desired rotation", () -> this.isAtSetpoint(), null);
        builder.addDoubleProperty("desired rotation", () -> desiredPosition, null);
        builder.addDoubleProperty("current angle", () -> getCurrentAngle(), null);
        builder.addDoubleProperty("absolute Rotation", () -> getAbsoluteRotation(), null);
        builder.addDoubleProperty("distance turret desiredPos", () -> distanceHubTurret, null);
        super.initSendable(builder);
    }
}
