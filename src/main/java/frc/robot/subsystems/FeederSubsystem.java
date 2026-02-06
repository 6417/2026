package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.FridolinsMotor;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {
    private final FridoSparkMax singulatorMotor;
    private final FridolinsMotor feederMotor;
    // Beam break sensor wired to a roboRIO DIO (digital input) port.
    private final DigitalInput beamBreak;

    public FeederSubsystem() {
        singulatorMotor = new FridoSparkMax(Constants.Feeder.singulatorMotorId);
        feederMotor = new FridoSparkMax(Constants.Feeder.feederMotorId);
        // DIO is where simple on/off sensors (like a beam break) are connected.
        // The port number comes from Constants.Feeder.beamBreakDio.
        beamBreak = new DigitalInput(Constants.Feeder.beamBreakDio);

        // TODO: Check if motors are inverted.
        singulatorMotor.setInverted(Constants.Feeder.singulatorMotorInverted);
        feederMotor.setInverted(Constants.Feeder.feederMotorInverted);

        singulatorMotor.setIdleMode(Constants.Feeder.idleMode);
        feederMotor.setIdleMode(Constants.Feeder.idleMode);

        // Velocity PID only for feeder motor.
        feederMotor.setPID(Constants.Feeder.feederVelocityPid, Constants.Feeder.feederFeedForward);
    }


    @Override
    public void periodic() {
        double currentThreshold = Constants.Feeder.singulatorStallCurrentAmps;
        double rpmThreshold = Constants.Feeder.singulatorStallRpmThreshold;
        if (currentThreshold > 0.0 && rpmThreshold >= 0.0) {
            double currentAmps = singulatorMotor.getOutputCurrent();
            double rpm = Math.abs(singulatorMotor.getEncoderVelocity());
            if (currentAmps > currentThreshold && rpm < rpmThreshold) {
                singulatorMotor.stopMotor();
            }
        }
    }

    public void setPercent(double singulator, double feeder) {
        singulatorMotor.set(MathUtil.clamp(singulator, -1.0, 1.0));
        feederMotor.set(MathUtil.clamp(feeder, -1.0, 1.0));
    }

    public void setSingulatorPercent(double singulator) {
        singulatorMotor.set(MathUtil.clamp(singulator, -1.0, 1.0));
    }

    public void setFeederPercent(double feeder) {
        feederMotor.set(MathUtil.clamp(feeder, -1.0, 1.0));
    }

    public void setFeederVelocityRpm(double velocityRpm) {
        feederMotor.setVelocity(velocityRpm);
    }

    public void feed() {
        setPercent(Constants.Feeder.singulatorSpeed, Constants.Feeder.feederSpeed);
    }

    public void reverse() {
        setPercent(Constants.Feeder.outtakeSingulatorSpeed, Constants.Feeder.outtakeFeederSpeed);
    }

    public void stop() {
        singulatorMotor.stopMotor();
        feederMotor.stopMotor();
    }

    public void stopFeeder() {
        feederMotor.stopMotor();
    }
    public void stopSingulator() {
        singulatorMotor.stopMotor();
    }

    public boolean isBallDetected() {
        boolean raw = beamBreak.get();
        // active-low: sensor returns false when beam is broken (common for beam breaks).
        // active-high: sensor returns true when beam is broken.
        // beamBreakInverted = true means the sensor is active-high, so we should not invert.
        boolean beamBlocked = Constants.Feeder.beamBreakInverted ? raw : !raw;
        return beamBlocked;
    }
}
