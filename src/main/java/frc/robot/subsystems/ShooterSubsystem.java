package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.FridolinsMotor;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    // Top and bottom shooter motors.
    private final FridolinsMotor topMotor;
    private final FridolinsMotor bottomMotor;

    // PID controllers for RPM control (one per motor).
    private final PIDController topPid;
    private final PIDController bottomPid;
    // Feedforward for basic motor model (kS + kV).
    private final SimpleMotorFeedforward feedforward;

    // Cached targets for debugging/telemetry.
    private double targetTopRpm = 0.0;
    private double targetBottomRpm = 0.0;

    public ShooterSubsystem() {
        topMotor = new FridoSparkMax(Constants.Shooter.topMotorId);
        bottomMotor = new FridoSparkMax(Constants.Shooter.bottomMotorId);

        // TODO: Verify which motor needs inversion so both wheels feed the ball forward.
        topMotor.setInverted(Constants.Shooter.topMotorInverted);
        bottomMotor.setInverted(Constants.Shooter.bottomMotorInverted);

        topMotor.setIdleMode(Constants.Shooter.idleMode);
        bottomMotor.setIdleMode(Constants.Shooter.idleMode);

        // Same gains for both motors for now; split if needed later.
        topPid = new PIDController(Constants.Shooter.kP, Constants.Shooter.kI, Constants.Shooter.kD);
        bottomPid = new PIDController(Constants.Shooter.kP, Constants.Shooter.kI, Constants.Shooter.kD);
        // Feedforward is shared because the model is the same for both wheels.
        feedforward = new SimpleMotorFeedforward(Constants.Shooter.kS, Constants.Shooter.kV);
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

    public void run(double rpm) {
        // Convenience: same RPM on both wheels.
        run(rpm, rpm);
    }

    private void run(double topRpm, double bottomRpm) {
        // Clamp target RPMs if a max RPM is configured.
        targetTopRpm = clampRpm(topRpm);
        targetBottomRpm = clampRpm(bottomRpm);

        // Spark Max encoder velocity is RPM by default.
        double topOutput = calculateOutput(topPid, topMotor.getEncoderVelocity(), targetTopRpm);
        double bottomOutput = calculateOutput(bottomPid, bottomMotor.getEncoderVelocity(), targetBottomRpm);

        // PID + FF output is still a percent (-1..1) to send to the motor.
        topMotor.set(topOutput);
        bottomMotor.set(bottomOutput);
    }

    /**
     * Set the speeds of the shooter motors based on distance to the target.
     *
     * The final implementation should use measured data points and curve fitting
     * (or interpolation) to map distance -> (top RPM, bottom RPM).
     */
    public void shootFromDistance(double distanceMeters) {
        // Use interpolation tables to map distance -> RPMs.
        double topRpm = Constants.Shooter.topRpmTable.getOutput(distanceMeters);
        double bottomRpm = Constants.Shooter.bottomRpmTable.getOutput(distanceMeters);
        run(topRpm, bottomRpm);
    }

    private double clampRpm(double rpm) {
        // If maxRpm is zero, treat it as "no limit".
        if (Constants.Shooter.maxRpm > 0.0) {
            rpm = MathUtil.clamp(rpm, -Constants.Shooter.maxRpm, Constants.Shooter.maxRpm);
        }
        return rpm;
    }

    private double calculateOutput(PIDController pid, double currentRpm, double targetRpm) {
        // PID does the error correction, FF gives a baseline voltage for the target speed.
        double outputPid = pid.calculate(currentRpm, targetRpm);
        double outputFf = feedforward.calculate(targetRpm);
        double output = outputPid + outputFf;

        // Prevent output from going too negative (protects against harsh reverse).
        if (output <= Constants.Shooter.maxNegPower) {
            output = Constants.Shooter.maxNegPower;
        }

        // Final safety clamp to motor input range.
        return MathUtil.clamp(output, -1.0, 1.0);
    }
}
