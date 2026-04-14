package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {

    private final DigitalOutput beamBreakSender;
    private final DigitalInput beamBreak;
    FridoSparkMax indexerMotor;
    SparkMaxConfig motorConfig;

    public IndexerSubsystem() {
        beamBreakSender = new DigitalOutput(Constants.Indexer.beamBreakSenderDio);
        beamBreakSender.set(true);
        beamBreak = new DigitalInput(Constants.Indexer.beamBreakDio);

        indexerMotor = new FridoSparkMax(Constants.Indexer.motorID);
        motorConfig = new SparkMaxConfig();

        indexerMotor.setInverted(Constants.Indexer.motorInverted);
        indexerMotor.setIdleMode(Constants.Indexer.mode);

        motorConfig.closedLoop.p(Constants.Indexer.pid.kP, ClosedLoopSlot.kSlot0)
                .i(Constants.Indexer.pid.kI, ClosedLoopSlot.kSlot0)
                .d(Constants.Indexer.pid.kD, ClosedLoopSlot.kSlot0);

        FeedForwardConfig ffConfig = new FeedForwardConfig();
        ffConfig.kS(Constants.Indexer.ff.kS);
        ffConfig.kV(Constants.Indexer.ff.kV);

        motorConfig.closedLoop.feedForward.apply(ffConfig); // for custom feedforward values
        indexerMotor.asSparkMax().configure(motorConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Indexer/BeamBreak", isBallDetected());
        Logger.recordOutput("Indexer/BeamBreakRaw", beamBreak.get());
        Logger.recordOutput("Indexer/IndexerCurrent", indexerMotor.asSparkMax().getOutputCurrent(), Units.Amps);
        Logger.recordOutput("Indexer/IndexerRPM", indexerMotor.asSparkMax().getEncoder().getVelocity(), Units.RPM);
        Logger.recordOutput("Indexer/IndexerRPMSetpoint",
                indexerMotor.asSparkMax().getClosedLoopController().getSetpoint(), Units.RPM);
    }

    public void stop() {
        indexerMotor.stopMotor();
    }

    public void setPercent(double p) {
        // Open-loop control (no PID). Useful for quick tests.
        double percent = MathUtil.clamp(p, -1.0, 1.0);
        indexerMotor.set(percent);
    }

    public void run(double topRpm) {
        // Velocity control takes RPM as input
        indexerMotor.asSparkMax().getClosedLoopController().setSetpoint(topRpm, ControlType.kVelocity);
    }

    public boolean isBallDetected() {
        boolean raw = beamBreak.get();
        boolean beamBlocked = Constants.Indexer.beamBreakInverted ? raw : !raw; // If the sensor is inverted, then a raw value of true means the beam is blocked. Otherwise, a raw value of false means the beam is blocked.
        return beamBlocked;
    }
}
