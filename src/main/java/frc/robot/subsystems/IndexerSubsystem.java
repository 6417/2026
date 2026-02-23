package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.FridolinsMotor;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {
    private final FridoSparkMax feederMotor;
    private final FridolinsMotor indexerMotor;
    // Beam break sensor wired to a roboRIO DIO (digital input) port.
    private final DigitalInput beamBreak;
    private double targetIndexerRpm = 0.0;

    public IndexerSubsystem() {
        feederMotor = new FridoSparkMax(Constants.Indexer.feederMotorId);
        indexerMotor = new FridoSparkMax(Constants.Indexer.indexerMotorId);
        // DIO is where simple on/off sensors (like a beam break) are connected.
        // The port number comes from Constants.Indexer.beamBreakDio.
        beamBreak = new DigitalInput(Constants.Indexer.beamBreakDio);

        // TODO: Check if motors are inverted.
        feederMotor.setInverted(Constants.Indexer.feederMotorInverted);
        indexerMotor.setInverted(Constants.Indexer.indexerMotorInverted);

        feederMotor.setIdleMode(Constants.Indexer.idleMode);
        indexerMotor.setIdleMode(Constants.Indexer.idleMode);

        // Velocity PID only for indexer motor.
        indexerMotor.setPID(Constants.Indexer.indexerVelocityPid, Constants.Indexer.indexerFeedForward);
    }


    @Override
    public void periodic() {
        double currentAmps = feederMotor.getOutputCurrent();
        double rpm = Math.abs(feederMotor.getEncoderVelocity());
        if (currentAmps > Constants.Indexer.feederStallCurrentAmps && rpm < Constants.Indexer.feederStallRpmThreshold) {
            feederMotor.stopMotor();
        }
    }

    public void setPercent(double feeder, double indexer) {
        feederMotor.set(MathUtil.clamp(feeder, -1.0, 1.0));
        indexerMotor.set(MathUtil.clamp(indexer, -1.0, 1.0));
    }

    public void setFeederPercent(double feeder) {
        feederMotor.set(MathUtil.clamp(feeder, -1.0, 1.0));
    }

    public void setIndexerPercent(double indexer) {
        indexerMotor.set(MathUtil.clamp(indexer, -1.0, 1.0));
    }

    public void setIndexerVelocityRpm(double velocityRpm) {
        targetIndexerRpm = velocityRpm;
        indexerMotor.setVelocity(velocityRpm);
    }

    public double getIndexerVelocityRpm() {
        return indexerMotor.getEncoderVelocity();
    }

    public double getTargetIndexerRpm() {
        return targetIndexerRpm;
    }

    public boolean isIndexerAtTargetRpm(double toleranceRpm) {
        return Math.abs(getIndexerVelocityRpm() - targetIndexerRpm) <= toleranceRpm;
    }

    public void feed() {
        setIndexerVelocityRpm(Constants.Indexer.indexingRpm);
    }

    public void emptyIndexer() {
        setIndexerPercent(Constants.Indexer.outtakeIndexerSpeed);
    }

    public void reverseFeeder() {
        setFeederPercent(Constants.Indexer.outtakeFeederSpeed);
    }

    public void stop() {
        feederMotor.stopMotor();
        indexerMotor.stopMotor();
    }

    public void stopIndexer() {
        targetIndexerRpm = 0.0;
        indexerMotor.stopMotor();
    }
    public void stopFeeder() {
        feederMotor.stopMotor();
    }

    public boolean isBallDetected() {
        boolean raw = beamBreak.get();
        // active-low: sensor returns false when beam is broken (common for beam breaks).
        // active-high: sensor returns true when beam is broken.
        // beamBreakInverted = true means the sensor is active-high, so we should not invert.
        boolean beamBlocked = Constants.Indexer.beamBreakInverted ? raw : !raw;
        return beamBlocked;
    }
}
