package frc.robot.diagnostics;

import frc.robot.subsystems.IntakeSubsystem;

/**
 * Diagnostic check for the intake motor.
 *
 * Runs the intake at its configured intake speed for a fixed duration and
 * samples current draw and encoder RPM. Stall detection in IntakeSubsystem
 * will still fire if a real stall occurs (which would itself be a FAIL signal).
 *
 * PASS : current ≥ MIN_CURRENT_A  AND  RPM ≥ MIN_RPM
 *        (motor spinning and drawing load — mechanically connected and responsive)
 * WARN : RPM ≥ MIN_RPM  but  current < MIN_CURRENT_A
 *        (motor spinning freely — possible mechanical disconnect / no load)
 * FAIL : RPM < MIN_RPM
 *        (no velocity response — motor not connected or driver fault)
 */
public class IntakeDiagnostic extends DiagnosticCommand {

    // A healthy spinning intake draws ~3–30 A; stall is defined at 80 A in Constants.
    private static final double MIN_CURRENT_A = 2.0;
    // Expect well above the stall threshold (200 RPM) when running freely
    private static final double MIN_RPM       = 500.0;
    private static final double DURATION_SECONDS = 1.5;

    private final IntakeSubsystem intake;

    // Running max values sampled during the check
    private double maxCurrentAmps = 0.0;
    private double maxRPM         = 0.0;

    public IntakeDiagnostic(IntakeSubsystem intake, DiagnosticReport report) {
        super("Intake/Motor", report, DURATION_SECONDS);
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    protected void onInitialize() {
        maxCurrentAmps = 0.0;
        maxRPM = 0.0;
        // Reset stall flag so the diagnostic can actually spin the motor
        intake.motorIsBlocked = false;
        intake.operatorIsControlling = false;
        intake.ballsIn();
    }

    @Override
    protected void onSample() {
        // IntakeSubsystem.periodic() may call stop() if it detects a stall —
        // keep commanding intake so we sample the true running state.
        if (!intake.motorIsBlocked) {
            intake.ballsIn();
        }

        double current = intake.getMotorCurrentAmps();
        double rpm     = Math.abs(intake.getMotorRPM());

        if (current > maxCurrentAmps) maxCurrentAmps = current;
        if (rpm     > maxRPM)         maxRPM         = rpm;
    }

    @Override
    protected void onEnd() {
        intake.stop();
        intake.operatorIsControlling = false;
    }

    @Override
    protected DiagnosticResult evaluate() {
        String details = String.format(
                "max RPM = %.0f (min %.0f), max current = %.1f A (min %.1f A)",
                maxRPM, MIN_RPM, maxCurrentAmps, MIN_CURRENT_A);

        if (maxRPM < MIN_RPM) {
            return DiagnosticResult.fail(checkName, details);
        } else if (maxCurrentAmps < MIN_CURRENT_A) {
            return DiagnosticResult.warn(checkName, details);
        } else {
            return DiagnosticResult.pass(checkName, details);
        }
    }
}
