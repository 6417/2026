package frc.robot.diagnostics;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Accumulates DiagnosticResults from all checks and publishes them to
 * SmartDashboard (under "Diagnostics/...") and the AdvantageKit Logger.
 *
 * Call {@link #record(DiagnosticResult)} from each DiagnosticCommand's end()
 * to register a result. The overall PASS/WARN/FAIL status updates automatically.
 */
public class DiagnosticReport {

    private final List<DiagnosticResult> results = new ArrayList<>();

    /** Register a result and push it to SmartDashboard + AK Logger immediately. */
    public void record(DiagnosticResult result) {
        results.add(result);

        String key = "Diagnostics/" + result.checkName();
        SmartDashboard.putString(key, result.status() + ": " + result.details());
        Logger.recordOutput(key + "/Status", result.status().name());
        Logger.recordOutput(key + "/Details", result.details());

        updateOverall();
    }

    /** Read-only view of all recorded results so far. */
    public List<DiagnosticResult> getResults() {
        return Collections.unmodifiableList(results);
    }

    /** True if every recorded result has status PASS. */
    public boolean allPassed() {
        return results.stream().allMatch(r -> r.status() == DiagnosticResult.Status.PASS);
    }

    private void updateOverall() {
        boolean anyFail = results.stream().anyMatch(r -> r.status() == DiagnosticResult.Status.FAIL);
        boolean anyWarn = results.stream().anyMatch(r -> r.status() == DiagnosticResult.Status.WARN);
        String overall = anyFail ? "FAIL" : anyWarn ? "WARN" : "PASS";

        SmartDashboard.putString("Diagnostics/Overall", overall);
        Logger.recordOutput("Diagnostics/Overall", overall);
    }
}
