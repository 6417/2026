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

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkFlex;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.FridolinsMotor;
import frc.robot.Constants;
import frc.robot.utils.LoggedTunableNumber;

public class ShooterSubsystem extends SubsystemBase {
    // Top and bottom shooter motors.
    private final FridoSparkFlex topMotor;
    private final FridoSparkFlex bottomMotor;

    SparkFlexConfig motorConfigTop;
    SparkFlexConfig motorConfigBottom;

    // Tunable RPMs — adjustable live from the dashboard when TUNING_MODE is on.
    private final LoggedTunableNumber tuneTopRpm =
        new LoggedTunableNumber("Shooter/TuneTopRPM", Constants.Shooter.defaultRPM);
    private final LoggedTunableNumber tuneBottomRpm =
        new LoggedTunableNumber("Shooter/TuneBottomRPM", Constants.Shooter.defaultRPM);

    // Last commanded setpoints, logged every periodic cycle.
    private double topRpmSetpoint = 0;
    private double bottomRpmSetpoint = 0;

    public ShooterSubsystem() {
        topMotor = new FridoSparkFlex(Constants.Shooter.topMotorId);
        bottomMotor = new FridoSparkFlex(Constants.Shooter.bottomMotorId);

        motorConfigTop = new SparkFlexConfig();
        motorConfigBottom = new SparkFlexConfig();

        topMotor.setIdleMode(Constants.Shooter.idleMode);
        bottomMotor.setIdleMode(Constants.Shooter.idleMode);

        motorConfigBottom.inverted(Constants.Shooter.bottomMotorInverted);
        motorConfigTop.inverted(Constants.Shooter.topMotorInverted);

        motorConfigTop.closedLoop.p(Constants.Shooter.pidBoth.kP, ClosedLoopSlot.kSlot0).i(Constants.Shooter.pidBoth.kI, ClosedLoopSlot.kSlot0)
            .d(Constants.Shooter.pidBoth.kD, ClosedLoopSlot.kSlot0);

        motorConfigBottom.closedLoop.p(Constants.Shooter.pidBoth.kP, ClosedLoopSlot.kSlot0).i(Constants.Shooter.pidBoth.kI, ClosedLoopSlot.kSlot0)
            .d(Constants.Shooter.pidBoth.kD, ClosedLoopSlot.kSlot0);
            
        FeedForwardConfig ffConfig = new FeedForwardConfig();
        ffConfig.kS(Constants.Shooter.ffTop.kS);
        ffConfig.kV(Constants.Shooter.ffTop.kV);
        motorConfigTop.closedLoop.feedForward.apply(ffConfig); // for custom feedforward values

        FeedForwardConfig ffConfigBottom = new FeedForwardConfig();
        ffConfigBottom.kS(Constants.Shooter.ffBottom.kS);
        ffConfigBottom.kV(Constants.Shooter.ffBottom.kV);
        motorConfigBottom.closedLoop.feedForward.apply(ffConfigBottom); // for custom feedforward values

        topMotor.asSparkFlex().configure(motorConfigTop, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        bottomMotor.asSparkFlex().configure(motorConfigBottom, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void stop() {
        topMotor.stopMotor();
        bottomMotor.stopMotor();
    }

    public void setPercent(double topPercent, double bottomPercent) {
        // Open-loop control (no PID). Useful for quick tests.
        double top = MathUtil.clamp(topPercent, -1.0, 1.0);
        double bottom = MathUtil.clamp(bottomPercent, -1.0, 1.0);
        topMotor.set(top);
        bottomMotor.set(bottom);
    }

    public void run(double topRpm, double bottomRpm) {
        topRpm = clampRpm(topRpm);
        bottomRpm = clampRpm(bottomRpm);
        topRpmSetpoint = topRpm;
        bottomRpmSetpoint = bottomRpm;
        // velocity control takes RPM as input
        topMotor.asSparkFlex().getClosedLoopController().setSetpoint(topRpm, ControlType.kVelocity);
        bottomMotor.asSparkFlex().getClosedLoopController().setSetpoint(bottomRpm, ControlType.kVelocity);
    }

    /**
     * Set the speeds of the shooter motors based on distance to the target.
     *
     * The final implementation should use measured data points and curve fitting
     * (or interpolation) to map distance -> (top RPM, bottom RPM).
     */
    public void shootFromDistance(double distanceMeters) {
        double topRpm, bottomRpm;
        if (Constants.TUNING_MODE) {
            // Read live from dashboard — adjust without redeploying.
            topRpm = tuneTopRpm.get();
            bottomRpm = tuneBottomRpm.get();
        } else {
            topRpm = Constants.Shooter.topRpmTable.getOutput(distanceMeters);
            bottomRpm = Constants.Shooter.bottomRpmTable.getOutput(distanceMeters);
        }
        run(topRpm, bottomRpm);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/TopRPM", topMotor.asSparkFlex().getEncoder().getVelocity());
        Logger.recordOutput("Shooter/BottomRPM", bottomMotor.asSparkFlex().getEncoder().getVelocity());
        Logger.recordOutput("Shooter/TopRPMSetpoint", topRpmSetpoint);
        Logger.recordOutput("Shooter/BottomRPMSetpoint", bottomRpmSetpoint);
        Logger.recordOutput("Shooter/TuningMode", Constants.TUNING_MODE);
    }

    private double clampRpm(double rpm) {
        /**
         * Clamp target RPM to a safe range.
         *
         * If maxRpm is set to 0, we treat it as "no limit" and return the input.
         * This avoids accidentally limiting the shooter when maxRpm is not configured yet.
         */
        if (Constants.Shooter.maxRpm > 0.0) {
            rpm = MathUtil.clamp(rpm, -Constants.Shooter.maxRpm, Constants.Shooter.maxRpm);
        }
        return rpm;
    }
}
