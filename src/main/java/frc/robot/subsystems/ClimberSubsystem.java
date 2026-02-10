package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    public enum ClimberState {
        LOW,
        MID,
        HIGH
    }

    private final FridoSparkMax climberMotor;
    private ClimberState targetState = ClimberState.LOW;

    public ClimberSubsystem() {
        // Initialize motor and apply configuration from Constants.
        climberMotor = new FridoSparkMax(Constants.Climber.motorId);
        climberMotor.setInverted(Constants.Climber.motorInverted);
        climberMotor.setIdleMode(Constants.Climber.idleMode);

        // Configure MAXMotion and PID slots (extend/retract).
        reconfigure();

        // Start with a known encoder reference.
        climberMotor.setEncoderPosition(Constants.Climber.resetEncoderPosition);
    }

    @Override
    public void periodic() {
    }

    public void setTargetState(ClimberState state) {
        // Update target and immediately command Motion Magic to the new setpoint.
        targetState = state;
        moveToTargetState();
    }

    public ClimberState getTargetState() {
        return targetState;
    }

    public boolean atTargetState() {
        // Position error check around the current state setpoint.
        double error = Math.abs(climberMotor.getEncoderTicks() - getTargetPositionForState(targetState));
        return error <= Constants.Climber.positionTolerance;
    }

    public void setManualPercent(double percent) {
        // Manual open-loop control for testing or encoder zeroing.
        climberMotor.set(MathUtil.clamp(percent, -1.0, 1.0));
    }

    public void stop() {
        climberMotor.stopMotor();
    }

    public void resetEncoder() {
        // Reset encoder position to the configured zero.
        climberMotor.setEncoderPosition(Constants.Climber.resetEncoderPosition);
    }

    public double getAmperage() {
        // Used for stall detection during zeroing.
        return climberMotor.getOutputCurrent();
    }

    public void setPositionForward(double position) {
        // Use slot 0 for extend (out).
        climberMotor.asSparkMax().getClosedLoopController()
                .setSetpoint(0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        climberMotor.setPosition(position);
    }

    public void setPositionUnderLoad(double position) {
        // Use slot 1 for retract (in).
        climberMotor.asSparkMax().getClosedLoopController()
                .setSetpoint(0, ControlType.kPosition, ClosedLoopSlot.kSlot1);
        climberMotor.setPosition(position);
    }

    private double getTargetPositionForState(ClimberState state) {
        // Map state -> encoder position.
        switch (state) {
            case LOW:
                return Constants.Climber.lowPosition;
            case MID:
                return Constants.Climber.midPosition;
            case HIGH:
                return Constants.Climber.highPosition;
            default:
                return Constants.Climber.lowPosition;
        }
    }

    private void moveToTargetState() {
        // Decide slot based on direction (extend vs retract).
        double current = climberMotor.getEncoderTicks();
        double target = getTargetPositionForState(targetState);
        if (target >= current) {
            setPositionForward(target);
        } else {
            setPositionUnderLoad(target);
        }
    }

    private void reconfigure() {
        // Rebuild MAXMotion configuration and apply both PID slots.
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        MAXMotionConfig motionConfig = new MAXMotionConfig();

        // Slot 0 = extend (out) PID and feedforward.
        motorConfig.closedLoop.p(Constants.Climber.pidValuesOut.kP, ClosedLoopSlot.kSlot0)
                .i(Constants.Climber.pidValuesOut.kI, ClosedLoopSlot.kSlot0)
                .d(Constants.Climber.pidValuesOut.kD, ClosedLoopSlot.kSlot0)
                .outputRange(Constants.Climber.pidValuesOut.peakOutputReverse,
                        Constants.Climber.pidValuesOut.peakOutputForward, ClosedLoopSlot.kSlot0)
                .velocityFF(Constants.Climber.pidValuesOut.kF.orElse(0.0), ClosedLoopSlot.kSlot0);
        Constants.Climber.pidValuesOut.iZone.ifPresent(
                iZone -> motorConfig.closedLoop.iZone(iZone, ClosedLoopSlot.kSlot0));

        // Slot 1 = retract (in) PID and feedforward.
        motorConfig.closedLoop.p(Constants.Climber.pidValuesIn.kP, ClosedLoopSlot.kSlot1)
                .i(Constants.Climber.pidValuesIn.kI, ClosedLoopSlot.kSlot1)
                .d(Constants.Climber.pidValuesIn.kD, ClosedLoopSlot.kSlot1)
                .outputRange(Constants.Climber.pidValuesIn.peakOutputReverse,
                        Constants.Climber.pidValuesIn.peakOutputForward, ClosedLoopSlot.kSlot1)
                .velocityFF(Constants.Climber.pidValuesIn.kF.orElse(0.0), ClosedLoopSlot.kSlot1);
        Constants.Climber.pidValuesIn.iZone.ifPresent(
                iZone -> motorConfig.closedLoop.iZone(iZone, ClosedLoopSlot.kSlot1));

        // MAXMotion constraints for extend (slot 0).
        motionConfig.allowedProfileError(Constants.Climber.allowedClosedLoopErrorOut, ClosedLoopSlot.kSlot0);
        motionConfig.maxAcceleration(Constants.Climber.maxAccelerationOut, ClosedLoopSlot.kSlot0);
        motionConfig.cruiseVelocity(Constants.Climber.maxVelocityOut, ClosedLoopSlot.kSlot0);
        motionConfig.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot0);

        // MAXMotion constraints for retract (slot 1).
        motionConfig.allowedProfileError(Constants.Climber.allowedClosedLoopErrorIn, ClosedLoopSlot.kSlot1);
        motionConfig.maxAcceleration(Constants.Climber.maxAccelerationIn, ClosedLoopSlot.kSlot1);
        motionConfig.cruiseVelocity(Constants.Climber.maxVelocityIn, ClosedLoopSlot.kSlot1);
        motionConfig.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot1);

        // Apply MAXMotion config to the controller.
        motorConfig.closedLoop.maxMotion.apply(motionConfig);
        climberMotor.asSparkMax().configure(motorConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);
    }
}
