package frc.fridowpi.motors;

import java.util.Collection;
import java.util.Optional;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

import frc.fridowpi.motors.utils.FeedForwardValues;
import frc.fridowpi.motors.utils.PidValues;

public class FridoSparkFlex implements FridolinsMotor {
    private SparkFlex motorProxy;
    private SparkFlexConfig config;
    private PidValues currentPidConfiguration;
    private PidType currentPidType;
    private double pidSetpoint;

    public FridoSparkFlex(int deviceID) {
        config = new SparkFlexConfig();
        motorProxy = new SparkFlex(deviceID, MotorType.kBrushless);
    }

    private com.revrobotics.spark.config.SparkBaseConfig.IdleMode convertFromFridoIdleMode(IdleMode mode) {
        switch (mode) {
            case kBrake:
                return com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake;
            case kCoast:
                return com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast;
            default:
                return com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast;
        }
    }

    private boolean convertToInvertFromFridoDirectionType(DirectionType direction) {
        switch (direction) {
            case followMaster:
                return false;
            case invertMaster:
                return true;
            default:
                return true;
        }
    }

    private Type convertFromFridoLimitswitchPolarity(LimitSwitchPolarity polarity) {
        switch (polarity) {
            case kNormallyClosed:
                return Type.kNormallyClosed;
            case kNormallyOpen:
                return Type.kNormallyOpen;
            default:
                return Type.kNormallyClosed;
        }
    }

    public SparkFlex asSparkFlex() {
        return motorProxy;
    }

    @Override
    public void set(double speed) {
        motorProxy.set(speed);
    }

    @Override
    public double get() {
        return motorProxy.get();
    }

    @Override
    public void setInverted(boolean isInverted) {
        config.inverted(true);
        motorProxy.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public boolean getInverted() {
        return motorProxy.configAccessor.getInverted();
    }

    @Override
    public void disable() {
        motorProxy.disable();
    }

    @Override
    public void stopMotor() {
        motorProxy.stopMotor();
    }

    @Override
    public void setIdleMode(IdleMode type) {
        config.idleMode(convertFromFridoIdleMode(type));
        motorProxy.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void follow(FridolinsMotor master, DirectionType direction) {
        if (master instanceof FridoSparkFlex) {
            config.follow(((FridoSparkFlex) master).asSparkFlex(), convertToInvertFromFridoDirectionType(direction));
        } else if (master instanceof FridoSparkMax) {
            config.follow(((FridoSparkMax) master).asSparkMax(), convertToInvertFromFridoDirectionType(direction));
        } else {
            throw new UnsupportedOperationException("Unimplemented method 'follow other controller type'");
        }
        motorProxy.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void factoryDefault() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException(
                "Unimplemented method 'factoryDefault', not necessary anymore it is done automatically while configuring");
    }

    @Override
    public void setPID(PidValues pidValues) {
        currentPidConfiguration = pidValues;
        config.closedLoop.p(pidValues.kP).i(pidValues.kI).d(pidValues.kD).outputRange(pidValues.peakOutputReverse,
                pidValues.peakOutputForward);
        pidValues.iZone.ifPresent(iZone -> config.closedLoop.iZone(iZone));
        motorProxy.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setPID(PidValues pidValues, FeedForwardValues feedForwardValues) {
        currentPidConfiguration = pidValues;
        config.closedLoop.p(pidValues.kP).i(pidValues.kI).d(pidValues.kD).outputRange(pidValues.peakOutputReverse,
                pidValues.peakOutputForward).velocityFF(pidValues.kF.get());
        pidValues.iZone.ifPresent(iZone -> config.closedLoop.iZone(iZone));
        motorProxy.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public boolean pidAtTarget() {
        switch (currentPidType) {
            case position:
                return Math.abs(getEncoderTicks()-pidSetpoint) < currentPidConfiguration.tolerance.orElse(0.01);
            case velocity:
                return Math.abs(getEncoderVelocity()-pidSetpoint) < currentPidConfiguration.tolerance.orElse(0.01);
            default:
                return false;
        }
    }

    @Override
    public void setVelocity(double velocity) {
        currentPidType = PidType.velocity;
        pidSetpoint = velocity;
        motorProxy.getClosedLoopController().setReference(velocity, ControlType.kVelocity);
    }

    @Override
    public void setPosition(double position) {
        currentPidType = PidType.position;
        pidSetpoint = position;
        motorProxy.getClosedLoopController().setReference(position, ControlType.kPosition);
    }

    @Override
    public void setPidTarget(double value, PidType type) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPidTarget'");
    }

    @Override
    public void selectPidSlot(int slotIndex) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'selectPidSlot'");
    }

    @Override
    public void enableForwardLimitSwitch(LimitSwitchPolarity polarity, boolean enable) {
        config.limitSwitch.forwardLimitSwitchEnabled(enable);
        config.limitSwitch.forwardLimitSwitchType(convertFromFridoLimitswitchPolarity(polarity));
        motorProxy.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void enableReverseLimitSwitch(LimitSwitchPolarity polarity, boolean enable) {
        config.limitSwitch.reverseLimitSwitchEnabled(enable);
        config.limitSwitch.reverseLimitSwitchType(convertFromFridoLimitswitchPolarity(polarity));
        motorProxy.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public boolean isForwardLimitSwitchActive() {
        return motorProxy.configAccessor.limitSwitch.getForwardLimitSwitchEnabled();
    }

    @Override
    public boolean isReverseLimitSwitchActive() {
        return motorProxy.configAccessor.limitSwitch.getReverseLimitSwitchEnabled();
    }

    @Override
    public void configEncoder(FridoFeedBackDevice device, int countsPerRev) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configEncoder'");
    }

    @Override
    public void setAccelerationLimit(double maxAcceleration) {
        // This only works for maxmotion and not for normal control types
        config.closedLoop.maxMotion.maxAcceleration(maxAcceleration);
        motorProxy.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setEncoderDirection(boolean inverted) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setEncoderDirection'");
    }

    @Override
    public void setEncoderPosition(double position) {
        motorProxy.getEncoder().setPosition(position);
    }

    @Override
    public double getEncoderTicks() {
        return motorProxy.getEncoder().getPosition();
    }

    @Override
    public double getEncoderVelocity() {
        return motorProxy.getEncoder().getVelocity();
    }

    @Override
    public void configOpenLoopRamp(double rate) {
        config.openLoopRampRate(rate);
        motorProxy.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public double getPidTarget() {
        return pidSetpoint;
    }

}
