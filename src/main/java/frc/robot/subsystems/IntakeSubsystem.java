package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkFlex;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final FridoSparkFlex intakeMotor;

    public IntakeSubsystem() {
        intakeMotor = new FridoSparkFlex(Constants.Intake.intakeMotorId);

        intakeMotor.setIdleMode(Constants.Intake.idleMode);
        intakeMotor.setInverted(Constants.Intake.intakeMotorInverted);

        SparkFlexConfig motorConfig = new SparkFlexConfig();

        PidValues pidValues = Constants.Intake.pid;

        motorConfig.closedLoop.p(pidValues.kP, ClosedLoopSlot.kSlot0).i(pidValues.kI, ClosedLoopSlot.kSlot0)
            .d(pidValues.kD, ClosedLoopSlot.kSlot0)
            .outputRange(pidValues.peakOutputReverse, pidValues.peakOutputForward, ClosedLoopSlot.kSlot0);

        FeedForwardConfig ffConfig = new FeedForwardConfig();
        ffConfig.kS(Constants.Intake.ff.kS);
        ffConfig.kV(Constants.Intake.ff.kV);
        ffConfig.kA(Constants.Intake.ff.kA);
        motorConfig.closedLoop.feedForward.apply(ffConfig); // for custom feedforward values
        
        motorConfig.smartCurrentLimit(Constants.Intake.stallAmps, Constants.Intake.freeAmps);
        intakeMotor.asSparkFlex().configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        // Could be added back for auto-intake
        // setDefaultCommand(new IntakeCommand(this));
    }

    @Override
    public void periodic() {
        double currentAmps = intakeMotor.getOutputCurrent();
        double rpms = intakeMotor.getEncoderVelocity();
        Logger.recordOutput("Intake/Current", currentAmps);
        Logger.recordOutput("Intake/RPM_Motor", rpms);
    }

    public void ballsIn() {
        intakeMotor.asSparkFlex().getClosedLoopController().setSetpoint(Constants.Intake.intakeSpeedRPM, ControlType.kVelocity);
    }

    public void ballsOut() {
        intakeMotor.asSparkFlex().getClosedLoopController().setSetpoint(Constants.Intake.outtakeSpeedRPM, ControlType.kVelocity);
    }

    public void setPercent(double percent) {
        double clamped = MathUtil.clamp(percent, -1.0, 1.0);
        intakeMotor.set(clamped);
    }

    public void stop() {
        intakeMotor.stopMotor();
    }

    public double getCurrentOutput() {
        return intakeMotor.getOutputCurrent();    
    }
}
