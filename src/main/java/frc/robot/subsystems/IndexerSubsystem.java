package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfigAccessor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkFlex;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.FridolinsMotor;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {

    private final DigitalOutput beamBreakSender;
    private final DigitalInput beamBreak;
    FridoSparkMax indexerMotor;
    SparkMaxConfig motorConfig;

    public IndexerSubsystem() {
        indexerMotor = new FridoSparkMax(Constants.Indexer.motorID);
        beamBreakSender = new DigitalOutput(Constants.Indexer.beamBreakSenderDio);
        beamBreakSender.set(true);
        beamBreak = new DigitalInput(Constants.Indexer.beamBreakDio);
        motorConfig = new SparkMaxConfig();

        indexerMotor.setIdleMode(Constants.Indexer.mode);
        
        indexerMotor.setInverted(Constants.Indexer.motorInverted);

        motorConfig.closedLoop.p(Constants.Indexer.pid.kP, ClosedLoopSlot.kSlot0).i(Constants.Indexer.pid.kI, ClosedLoopSlot.kSlot0)
            .d(Constants.Indexer.pid.kD, ClosedLoopSlot.kSlot0);
            
        FeedForwardConfig ffConfig = new FeedForwardConfig();
        ffConfig.kS(Constants.Indexer.ff.kS);
        ffConfig.kV(Constants.Indexer.ff.kV);
        motorConfig.closedLoop.feedForward.apply(ffConfig); // for custom feedforward values
        indexerMotor.asSparkMax().configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Indexer/BeamBreak", isBallDetected());
        Logger.recordOutput("Indexer/BeamBreakRaw", beamBreak.get());
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
        // velocity control takes RPM as input
        indexerMotor.asSparkMax().getClosedLoopController().setSetpoint(topRpm, ControlType.kVelocity);
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
