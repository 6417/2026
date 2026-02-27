package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {
    private FridoSparkMax feederMotor;
    private SparkMaxConfig motorConfig;
    public FeederSubsystem() {
        feederMotor = new FridoSparkMax(Constants.Feeder.motorId);

        // TODO: Check if motor is inverted.
        feederMotor.setInverted(Constants.Feeder.motorInverted);

        motorConfig = new SparkMaxConfig();

        feederMotor.setIdleMode(Constants.Feeder.idleMode);

        motorConfig.closedLoop.p(Constants.Feeder.pid.kP, ClosedLoopSlot.kSlot0).i(Constants.Feeder.pid.kI, ClosedLoopSlot.kSlot0)
            .d(Constants.Feeder.pid.kD, ClosedLoopSlot.kSlot0);
            
        FeedForwardConfig ffConfig = new FeedForwardConfig();
        ffConfig.kS(Constants.Feeder.ff.kS);
        ffConfig.kV(Constants.Feeder.ff.kV);
        motorConfig.closedLoop.feedForward.apply(ffConfig); // for custom feedforward values
        feederMotor.asSparkMax().configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
    }

    public void run(double topRpm) {
        // velocity control takes RPM as input
        feederMotor.asSparkMax().getClosedLoopController().setSetpoint(topRpm, ControlType.kVelocity);
    }

    public void setPercent(double feeder, double indexer) {
        feederMotor.set(MathUtil.clamp(feeder, -1.0, 1.0));
    }

    public void stop() {
        feederMotor.stopMotor();
    }
}