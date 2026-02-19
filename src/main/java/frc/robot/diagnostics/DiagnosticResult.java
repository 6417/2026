package frc.robot.diagnostics;

/**
 * Immutable result of a single diagnostic check.
 * Holds the check name, pass/warn/fail status, and a human-readable details string.
 */
public record DiagnosticResult(String checkName, Status status, String details) {

    public enum Status { PASS, WARN, FAIL }

    /** Convenience factory for a passing result. */
    public static DiagnosticResult pass(String checkName, String details) {
        return new DiagnosticResult(checkName, Status.PASS, details);
    }

    /** Convenience factory for a warning result. */
    public static DiagnosticResult warn(String checkName, String details) {
        return new DiagnosticResult(checkName, Status.WARN, details);
    }

    /** Convenience factory for a failing result. */
    public static DiagnosticResult fail(String checkName, String details) {
        return new DiagnosticResult(checkName, Status.FAIL, details);
    }
}
