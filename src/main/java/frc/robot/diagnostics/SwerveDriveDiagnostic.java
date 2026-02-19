package frc.robot.diagnostics;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Diagnostic check for the swerve drive motors.
 *
 * Commands a small forward chassis speed and verifies that the measured
 * robot velocity responds within expected bounds.
 *
 * PASS : measured vx ≥ MIN_PASS_SPEED_MPS   (drive motors responding normally)
 * WARN : measured vx ≥ MIN_WARN_SPEED_MPS   (weak response — check motors/encoders)
 * FAIL : measured vx < MIN_WARN_SPEED_MPS   (no measurable drive response)
 *
 * Safety: the robot will move forward a few centimetres — ensure clear space ahead.
 */
public class SwerveDriveDiagnostic extends DiagnosticCommand {

    private static final double COMMANDED_SPEED_MPS = 0.5;
    private static final double MIN_PASS_SPEED_MPS  = 0.25; // 50 % of commanded
    private static final double MIN_WARN_SPEED_MPS  = 0.10; // 20 % of commanded
    private static final double DURATION_SECONDS    = 1.5;

    private final SwerveSubsystem swerve;
    private double maxMeasuredVx = 0.0;

    public SwerveDriveDiagnostic(SwerveSubsystem swerve, DiagnosticReport report) {
        super("Swerve/DriveMotors", report, DURATION_SECONDS);
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    protected void onInitialize() {
        maxMeasuredVx = 0.0;
        swerve.setAutomatedControl(); // Prevent periodic joystick commands from overriding us
    }

    @Override
    protected void onSample() {
        // Command forward motion every loop so motor controllers stay active
        swerve.setChassisSpeeds(new ChassisSpeeds(COMMANDED_SPEED_MPS, 0, 0));

        double vx = Math.abs(swerve.getRobotVelocity().vxMetersPerSecond);
        if (vx > maxMeasuredVx) {
            maxMeasuredVx = vx;
        }
    }

    @Override
    protected void onEnd() {
        swerve.setChassisSpeeds(new ChassisSpeeds()); // Stop the robot
    }

    @Override
    protected DiagnosticResult evaluate() {
        String details = String.format("max vx = %.2f m/s (commanded %.2f m/s)",
                maxMeasuredVx, COMMANDED_SPEED_MPS);

        if (maxMeasuredVx >= MIN_PASS_SPEED_MPS) {
            return DiagnosticResult.pass(checkName, details);
        } else if (maxMeasuredVx >= MIN_WARN_SPEED_MPS) {
            return DiagnosticResult.warn(checkName, details);
        } else {
            return DiagnosticResult.fail(checkName, details);
        }
    }
}
