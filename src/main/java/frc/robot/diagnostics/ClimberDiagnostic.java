package frc.robot.diagnostics;

import frc.robot.subsystems.ClimberSubsystem;

/**
 * Diagnostic check for basic climber motor response.
 *
 * The test drives the climber slowly in open-loop for a short duration and
 * verifies encoder movement plus current draw.
 *
 * PASS : position delta >= MIN_DELTA_TICKS and current >= MIN_CURRENT_A
 * WARN : position delta >= MIN_DELTA_TICKS but current < MIN_CURRENT_A
 * FAIL : position delta < MIN_DELTA_TICKS
 */
public class ClimberDiagnostic extends DiagnosticCommand {

    private static final double TEST_PERCENT = 0.15;
    private static final double DURATION_SECONDS = 1.2;
    private static final double MIN_DELTA_TICKS = 0.8;
    private static final double MIN_CURRENT_A = 1.5;

    private final ClimberSubsystem climber;
    private double startTicks = 0.0;
    private double maxDeltaTicks = 0.0;
    private double maxCurrentAmps = 0.0;

    public ClimberDiagnostic(ClimberSubsystem climber, DiagnosticReport report) {
        super("Climber/Motor", report, DURATION_SECONDS);
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    protected void onInitialize() {
        startTicks = climber.getPositionTicks();
        maxDeltaTicks = 0.0;
        maxCurrentAmps = 0.0;
        climber.setManualPercent(TEST_PERCENT);
    }

    @Override
    protected void onSample() {
        climber.setManualPercent(TEST_PERCENT);

        double delta = Math.abs(climber.getPositionTicks() - startTicks);
        if (delta > maxDeltaTicks) {
            maxDeltaTicks = delta;
        }

        double current = climber.getAmperage();
        if (current > maxCurrentAmps) {
            maxCurrentAmps = current;
        }
    }

    @Override
    protected void onEnd() {
        climber.stop();
    }

    @Override
    protected DiagnosticResult evaluate() {
        String details = String.format(
                "max delta = %.3f ticks (min %.3f), max current = %.2f A (min %.2f A)",
                maxDeltaTicks,
                MIN_DELTA_TICKS,
                maxCurrentAmps,
                MIN_CURRENT_A);

        if (maxDeltaTicks < MIN_DELTA_TICKS) {
            return DiagnosticResult.fail(checkName, details);
        } else if (maxCurrentAmps < MIN_CURRENT_A) {
            return DiagnosticResult.warn(checkName, details);
        }
        return DiagnosticResult.pass(checkName, details);
    }
}
