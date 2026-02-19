package frc.robot.diagnostics;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Abstract base for all diagnostic checks.
 *
 * Lifecycle:
 *   initialize() → onInitialize()   — actuate / set up the check
 *   execute()    → onSample()       — read sensors each loop
 *   isFinished() → timer elapsed
 *   end()        → onEnd() + evaluate() → DiagnosticResult recorded in report
 *
 * Subclasses must:
 *   - Call addRequirements(...) in their constructor for any subsystems they use.
 *   - Implement onInitialize(), onSample(), onEnd(), and evaluate().
 */
public abstract class DiagnosticCommand extends Command {

    protected final String checkName;
    protected final DiagnosticReport report;
    protected final Timer timer = new Timer();
    private final double durationSeconds;

    protected DiagnosticCommand(String checkName, DiagnosticReport report, double durationSeconds) {
        this.checkName = checkName;
        this.report = report;
        this.durationSeconds = durationSeconds;
    }

    /** Called once when the command starts. Apply actuation here. */
    protected abstract void onInitialize();

    /** Called every loop while running. Sample sensor values here. */
    protected abstract void onSample();

    /** Called once before evaluate(). Stop actuation / clean up here. */
    protected abstract void onEnd();

    /**
     * Evaluate sampled data and return the check's result.
     * Only called if the command ran to completion (not interrupted).
     */
    protected abstract DiagnosticResult evaluate();

    // ── WPILib lifecycle (final to enforce the template pattern) ──────────────

    @Override
    public final void initialize() {
        timer.restart();
        onInitialize();
    }

    @Override
    public final void execute() {
        onSample();
    }

    @Override
    public final boolean isFinished() {
        return timer.hasElapsed(durationSeconds);
    }

    @Override
    public final void end(boolean interrupted) {
        onEnd();
        DiagnosticResult result = interrupted
                ? DiagnosticResult.fail(checkName, "Test interrupted before completion")
                : evaluate();
        report.record(result);
    }
}
