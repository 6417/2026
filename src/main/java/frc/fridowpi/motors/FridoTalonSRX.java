package frc.fridowpi.motors;

import java.util.Collection;
import java.util.List;
import java.util.Optional;

import org.ejml.simple.UnsupportedOperation;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.fridowpi.motors.utils.FeedForwardValues;
import frc.fridowpi.motors.utils.PidValues;

public class FridoTalonSRX extends WPI_TalonSRX implements FridolinsMotor {

    private Optional<PidValues> pidValues = Optional.empty();
    private double target = 0.0;

    public FridoTalonSRX(int deviceID) {
        super(deviceID);
    }

    @Override
    public void set(double speed){
        super.set(speed);
    }

    @Override
    public void setPosition(double position) {
        super.set(ControlMode.Position, position);
		target = position;
    }

    @Override
    public void setVelocity(double velocity) {
        super.set(ControlMode.Velocity, velocity);
    }

    private LimitSwitchNormal convertFromFridoLimitSwitchPolarity(LimitSwitchPolarity polarity) {
        switch (polarity) {
            case kNormallyOpen:
                return LimitSwitchNormal.NormallyOpen;
            case kNormallyClosed:
                return LimitSwitchNormal.NormallyClosed;
            default:
                return LimitSwitchNormal.Disabled;
        }
    }

    @Override
    public void enableForwardLimitSwitch(LimitSwitchPolarity polarity, boolean enable) {
        if (!enable) {
            polarity = LimitSwitchPolarity.kDisabled;
        }
        super.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                convertFromFridoLimitSwitchPolarity(polarity));
    }

    @Override
    public void enableReverseLimitSwitch(LimitSwitchPolarity polarity, boolean enable) {
        if (!enable) {
            polarity = LimitSwitchPolarity.kDisabled;
        }
        super.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                convertFromFridoLimitSwitchPolarity(polarity));
    }

    @Override
    public boolean isForwardLimitSwitchActive() {
        return getSensorCollection().isFwdLimitSwitchClosed();
    }

    @Override
    public boolean isReverseLimitSwitchActive() {
        return getSensorCollection().isRevLimitSwitchClosed();
    }

    private NeutralMode convertFromFridoIdleMode(IdleMode type) {
        switch (type) {
            case kBrake:
                return NeutralMode.Brake;
            case kCoast:
                return NeutralMode.Coast;
            default:
                return NeutralMode.Brake;
        }
    }

    private InvertType convertFromFridoDirectionsType(FridolinsMotor.DirectionType direction) {
        switch (direction) {
            case followMaster:
                return InvertType.FollowMaster;
            case invertMaster:
                return InvertType.OpposeMaster;
            default:
                return InvertType.FollowMaster;
        }
    }

    @Override
    public void setIdleMode(IdleMode type) {
        super.setNeutralMode(convertFromFridoIdleMode(type));
    }

    @Override
    public void follow(FridolinsMotor master, DirectionType direction) {
        if (master instanceof FridoTalonSRX) {
            super.follow((FridoTalonSRX) master);
            super.setInverted(convertFromFridoDirectionsType(direction));
        }
    }

    @Override
    public void setInverted(boolean forward) {
        super.setInverted(forward);
    }

    @Override
    public void setEncoderDirection(boolean inverted) {
        super.setSensorPhase(inverted);
    }

    @Override
    public void setEncoderPosition(double position) {
        super.setSelectedSensorPosition((int) position);
    }

    @Override
    public double getEncoderTicks() {
        return super.getSelectedSensorPosition();
    }

    @Override
    public void factoryDefault() {
        super.configFactoryDefault();
    }

    @Override
    public void configOpenLoopRamp(double rate) {
        super.configOpenloopRamp(rate);

    }

    private com.ctre.phoenix.motorcontrol.FeedbackDevice convertFromFridoFeedbackDevice(
            FridoFeedBackDevice device) {
        switch (device) {
            case kRelative:
                return com.ctre.phoenix.motorcontrol.FeedbackDevice.QuadEncoder;
            default:
                throw new Error("Unimplemented");
        }
    }

    @Override
    public void configEncoder(FridoFeedBackDevice device, int countsPerRev) {
        super.configSelectedFeedbackSensor(convertFromFridoFeedbackDevice(device));
    }

    @Override
    public void setPID(PidValues pidValues) {
        if (pidValues.slotIdX.isPresent()) {
            super.config_kP(pidValues.slotIdX.get(), pidValues.kP);
            super.config_kI(pidValues.slotIdX.get(), pidValues.kI);
            super.config_kD(pidValues.slotIdX.get(), pidValues.kD);
            super.configPeakOutputForward(pidValues.peakOutputForward);
            super.configPeakOutputReverse(pidValues.peakOutputReverse);
            pidValues.iZone.ifPresent(iZone -> super.config_IntegralZone(pidValues.slotIdX.get(), iZone));
            pidValues.cruiseVelocity.ifPresent((cruiseVelocity) -> super.configMotionCruiseVelocity((int) cruiseVelocity.doubleValue()));
            pidValues.acceleration.ifPresent((acceleration) -> super.configMotionAcceleration((int) acceleration.doubleValue()));
            pidValues.kF.ifPresent((kF) -> super.config_kF(pidValues.slotIdX.get(), kF));
            pidValues.tolerance.ifPresent((t) -> super.configAllowableClosedloopError(pidValues.slotIdX.get(), t));
            super.selectProfileSlot(pidValues.slotIdX.get(), 0);
        } else {
            try {
                throw new Exception("You have to give a slotID for TalonSRX pidControllers");
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
    
	@Override
	public void setPID(PidValues pidValues, FeedForwardValues feedForwardValues) {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'setPID'");
	}

    @Override
    public double getEncoderVelocity() {
        return super.getSelectedSensorVelocity();
    }

    @Override
    public boolean pidAtTarget() {
        return super.isMotionProfileFinished();
    }

    @Override
    public void setPidTarget(double value, PidType type) {
        switch (type) {
            case position:
                super.set(ControlMode.Position, value);
				target = value;
                break;
            case smartMotion:
                super.set(ControlMode.MotionMagic, value);
                break;
            case velocity:
                super.set(ControlMode.Velocity, value);
                break;
            case voltage:
                super.setVoltage(value);
                break;
            case smartVelocity:
            default:
                throw new UnsupportedOperation("Not possible");
        }
    }

    @Override
    public void selectPidSlot(int slotIndex) {
    }

    @Override
    public double getPidTarget() {
        return target;
    }

	@Override
	public void setAccelerationLimit(double maxAcceleration) {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'setAccelerationLimit'");
	}
}
