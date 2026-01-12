package frc.fridowpi.motors;

import frc.fridowpi.motors.utils.FeedForwardValues;
import frc.fridowpi.motors.utils.PidValues;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public interface FridolinsMotor extends MotorController {
    public enum IdleMode {
        kBrake, kCoast
    }

    public enum DirectionType {
        followMaster, invertMaster
    }

    public enum PidType {
        velocity,
        voltage,
        position,
        smartMotion,
        smartVelocity;
    }

    public void setIdleMode(IdleMode type);

    public void follow(FridolinsMotor master, DirectionType direction);

    public void factoryDefault();

    public void setPID(PidValues pidValues);

    public void setPID(PidValues pidValues, FeedForwardValues feedForwardValues);

    public boolean pidAtTarget();

    public void setVelocity(double velocity);

    public void setPosition(double position);

    public void setPidTarget(double value, PidType type);

    public void selectPidSlot(int slotIndex);

    public enum LimitSwitchPolarity {
        kNormallyOpen, kNormallyClosed, kDisabled
    }

    public void enableForwardLimitSwitch(LimitSwitchPolarity polarity, boolean enable);

    public void enableReverseLimitSwitch(LimitSwitchPolarity polarity, boolean enable);

    public boolean isForwardLimitSwitchActive();

    public boolean isReverseLimitSwitchActive();

    public enum FridoFeedBackDevice {
        kRelative, kAlternative, kBuildin, kAbsolute
    }

    public void configEncoder(FridoFeedBackDevice device, int countsPerRev);

    public void setAccelerationLimit(double maxAcceleration);

    public void setEncoderDirection(boolean inverted);

    public void setEncoderPosition(double position);

    public double getEncoderTicks();

    public double getEncoderVelocity();

    public void configOpenLoopRamp(double rate);

    public double getPidTarget();

    default LimitSwitch getForwardLimitSwitch() {
        return this::isForwardLimitSwitchActive;
    }

    default LimitSwitch getReverseLimitSwitch() {
        return this::isReverseLimitSwitchActive;
    }
}
