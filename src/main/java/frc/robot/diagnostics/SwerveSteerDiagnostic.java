package frc.robot.diagnostics;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Diagnostic check for the swerve steer (azimuth) motors.
 *
 * Commands a pure rotation (no translation) and verifies that the measured
 * angular velocity responds within expected bounds. Steer motors must rotate
 * each wheel to the correct tangential angle before the robot can spin —
 * a weak or absent omega reading points to steer motor / encoder issues.
 *
 * PASS : measured |omega| ≥ MIN_PASS_RAD_S   (steer motors responding normally)
 * WARN : measured |omega| ≥ MIN_WARN_RAD_S   (weak response — check steer chain)
 * FAIL : measured |omega| < MIN_WARN_RAD_S   (no measurable steer response)
 *
 * Safety: the robot will rotate in-place by ~0.5 rad (~30°) — ensure clear surroundings.
 */
public class SwerveSteerDiagnostic extends DiagnosticCommand {

    private static final double COMMANDED_OMEGA_RAD_S = 0.5;
    private static final double MIN_PASS_RAD_S        = 0.25; // 50 % of commanded
    private static final double MIN_WARN_RAD_S        = 0.10; // 20 % of commanded
    private static final double DURATION_SECONDS      = 1.5;

    private final SwerveSubsystem swerve;
    private double maxMeasuredOmega = 0.0;

    public SwerveSteerDiagnostic(SwerveSubsystem swerve, DiagnosticReport report) {
        super("Swerve/SteerMotors", report, DURATION_SECONDS);
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    protected void onInitialize() {
        maxMeasuredOmega = 0.0;
        swerve.setAutomatedControl();
    }

    @Override
    protected void onSample() {
        // Pure rotation command — steer motors must move wheels to tangential angles
        swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, COMMANDED_OMEGA_RAD_S));

        double omega = Math.abs(swerve.getRobotVelocity().omegaRadiansPerSecond);
        if (omega > maxMeasuredOmega) {
            maxMeasuredOmega = omega;
        }
    }

    @Override
    protected void onEnd() {
        swerve.setChassisSpeeds(new ChassisSpeeds()); // Stop rotation
    }

    @Override
    protected DiagnosticResult evaluate() {
        String details = String.format("max |omega| = %.3f rad/s (commanded %.2f rad/s)",
                maxMeasuredOmega, COMMANDED_OMEGA_RAD_S);

        if (maxMeasuredOmega >= MIN_PASS_RAD_S) {
            return DiagnosticResult.pass(checkName, details);
        } else if (maxMeasuredOmega >= MIN_WARN_RAD_S) {
            return DiagnosticResult.warn(checkName, details);
        } else {
            return DiagnosticResult.fail(checkName, details);
        }
    }
}
