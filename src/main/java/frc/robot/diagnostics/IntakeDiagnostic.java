package frc.robot.diagnostics;

import frc.robot.subsystems.IntakeSubsystem;

/**
 * Diagnostic check for the intake motor.
 *
 * Runs intake for a fixed duration and samples current and RPM.
 *
 * PASS : current >= MIN_CURRENT_A and RPM >= MIN_RPM
 * WARN : RPM >= MIN_RPM but current < MIN_CURRENT_A
 * FAIL : RPM < MIN_RPM
 */
public class IntakeDiagnostic extends DiagnosticCommand {

    private static final double MIN_CURRENT_A = 2.0;
    private static final double MIN_RPM = 500.0;
    private static final double DURATION_SECONDS = 1.5;

    private final IntakeSubsystem intake;

    private double maxCurrentAmps = 0.0;
    private double maxRPM = 0.0;

    public IntakeDiagnostic(IntakeSubsystem intake, DiagnosticReport report) {
        super("Intake/Motor", report, DURATION_SECONDS);
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    protected void onInitialize() {
        maxCurrentAmps = 0.0;
        maxRPM = 0.0;

        // Align with current IntakeSubsystem API.
        intake.run = true;
        intake.isIntakeOn = true;
        intake.intake();
    }

    @Override
    protected void onSample() {
        // Keep commanding intake unless subsystem stall logic has tripped run=false.
        if (intake.run) {
            intake.intake();
        }

        double current = intake.getMotorCurrentAmps();
        double rpm = Math.abs(intake.getMotorRPM());

        if (current > maxCurrentAmps) {
            maxCurrentAmps = current;
        }
        if (rpm > maxRPM) {
            maxRPM = rpm;
        }
    }

    @Override
    protected void onEnd() {
        intake.stop();
        intake.isIntakeOn = false;
    }

    @Override
    protected DiagnosticResult evaluate() {
        String details = String.format(
                "max RPM = %.0f (min %.0f), max current = %.1f A (min %.1f A)",
                maxRPM,
                MIN_RPM,
                maxCurrentAmps,
                MIN_CURRENT_A);

        if (maxRPM < MIN_RPM) {
            return DiagnosticResult.fail(checkName, details);
        } else if (maxCurrentAmps < MIN_CURRENT_A) {
            return DiagnosticResult.warn(checkName, details);
        }
        return DiagnosticResult.pass(checkName, details);
    }
}
