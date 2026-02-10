package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoFalcon500v6;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    public enum ClimberState {
        LOW,
        MID,
        HIGH
    }

    private final FridoFalcon500v6 climberMotor;
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0);
    private final MotionMagicConfigs motionMagicOut = new MotionMagicConfigs();
    private final MotionMagicConfigs motionMagicIn = new MotionMagicConfigs();
    private ClimberState targetState = ClimberState.LOW;

    public ClimberSubsystem() {
        // Initialize motor and apply configuration from Constants.
        climberMotor = new FridoFalcon500v6(Constants.Climber.motorId);
        climberMotor.setInverted(Constants.Climber.motorInverted);
        climberMotor.setIdleMode(Constants.Climber.idleMode);

        // Configure Motion Magic and PID slots (extend/retract).
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
        return climberMotor.asTalonFX().getStatorCurrent().getValueAsDouble();
    }

    public void setPositionForward(double position) {
        // Use slot 0 for extend (out) with outward motion constraints.
        applyMotionMagicConfig(motionMagicOut);
        motionMagicRequest.Position = position;
        motionMagicRequest.Slot = 0;
        climberMotor.asTalonFX().setControl(motionMagicRequest);
    }

    public void setPositionUnderLoad(double position) {
        // Use slot 1 for retract (in) with inward motion constraints.
        applyMotionMagicConfig(motionMagicIn);
        motionMagicRequest.Position = position;
        motionMagicRequest.Slot = 1;
        climberMotor.asTalonFX().setControl(motionMagicRequest);
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
        // Rebuild Motion Magic configuration and apply both PID slots.
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        // Slot 0 = extend (out) PID and feedforward.
        Slot0Configs slot0 = motorConfig.Slot0;
        slot0.kP = Constants.Climber.pidValuesOut.kP;
        slot0.kI = Constants.Climber.pidValuesOut.kI;
        slot0.kD = Constants.Climber.pidValuesOut.kD;
        slot0.kV = Constants.Climber.pidValuesOut.kF.orElse(0.0);

        // Slot 1 = retract (in) PID and feedforward.
        Slot1Configs slot1 = motorConfig.Slot1;
        slot1.kP = Constants.Climber.pidValuesIn.kP;
        slot1.kI = Constants.Climber.pidValuesIn.kI;
        slot1.kD = Constants.Climber.pidValuesIn.kD;
        slot1.kV = Constants.Climber.pidValuesIn.kF.orElse(0.0);

        // Motion Magic constraints for extend/retract (per-direction).
        motionMagicOut.MotionMagicCruiseVelocity = Constants.Climber.maxVelocityOut;
        motionMagicOut.MotionMagicAcceleration = Constants.Climber.maxAccelerationOut;
        motionMagicIn.MotionMagicCruiseVelocity = Constants.Climber.maxVelocityIn;
        motionMagicIn.MotionMagicAcceleration = Constants.Climber.maxAccelerationIn;

        // Apply base config and default Motion Magic config (extend).
        climberMotor.asTalonFX().getConfigurator().apply(motorConfig);
        applyMotionMagicConfig(motionMagicOut);
    }

    private void applyMotionMagicConfig(MotionMagicConfigs config) {
        // Update Motion Magic constraints without overwriting other configs.
        climberMotor.asTalonFX().getConfigurator().apply(config);
    }
}
