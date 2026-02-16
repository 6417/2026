package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Analytic moving-shot solver.
 *
 * <p>The solver computes the required turret yaw and muzzle speed directly from:
 * shooter pose, target pose, robot translational velocity, fixed launch pitch, and gravity.
 *
 * <p>This avoids using a global scale clamp as the primary mechanism. Any clamping is left to
 * hardware safety limits (RPM saturation) later in the control path.
 */
public final class ShotKinematicSolver {
    /** Status of the kinematic solve attempt. */
    public enum SolveStatus {
        SOLVED,
        TARGET_TOO_CLOSE,
        NO_VALID_FLIGHT_TIME,
        INVALID_INPUT
    }

    /** Input bundle for a solve call. */
    public record SolveRequest(
            Translation3d shooterPositionField,
            Translation3d targetPositionField,
            Translation2d robotVelocityFieldMps,
            double launchPitchRad,
            double gravityMetersPerSec2,
            double minFlightTimeSec,
            double maxFlightTimeSec,
            int maxIterations,
            double flightTimeToleranceSec,
            double verticalErrorToleranceMeters,
            double dragCoefficientPerMeter,
            double dragSpeedCompensationGain) {
    }

    /** Output bundle from the solver. */
    public record SolveResult(
            SolveStatus status,
            double flightTimeSec,
            double yawFieldRad,
            Translation2d muzzleVelocityFieldMps,
            double requiredMuzzleSpeedMps,
            double verticalResidualMeters) {
        public boolean solved() {
            return status == SolveStatus.SOLVED;
        }
    }

    private static final int BRACKET_SAMPLES = 80;

    private ShotKinematicSolver() {
    }

    public static SolveResult solve(SolveRequest request) {
        if (!isFinitePositive(request.gravityMetersPerSec2())
                || !isFinitePositive(request.minFlightTimeSec())
                || !isFinitePositive(request.maxFlightTimeSec())
                || request.maxFlightTimeSec() <= request.minFlightTimeSec()
                || request.maxIterations() <= 0
                || !Double.isFinite(request.launchPitchRad())) {
            return invalidResult();
        }

        Translation3d delta3d = request.targetPositionField().minus(request.shooterPositionField());
        Translation2d deltaXY = new Translation2d(delta3d.getX(), delta3d.getY());
        double horizontalDistance = deltaXY.getNorm();
        if (!Double.isFinite(horizontalDistance) || horizontalDistance < 1e-6) {
            return new SolveResult(
                    SolveStatus.TARGET_TOO_CLOSE,
                    0.0,
                    0.0,
                    new Translation2d(),
                    0.0,
                    Double.POSITIVE_INFINITY);
        }

        double cosPitch = Math.cos(request.launchPitchRad());
        if (Math.abs(cosPitch) < 1e-6) {
            return invalidResult();
        }

        double tanPitch = Math.tan(request.launchPitchRad());
        double targetDeltaZ = delta3d.getZ();

        Bracket bracket = findBracket(
                request,
                deltaXY,
                targetDeltaZ,
                tanPitch);

        double solvedTimeSec;
        double residualMeters;
        if (bracket.hasSignChange()) {
            SolveTimeResult timeResult = solveTimeBisection(
                    request,
                    deltaXY,
                    targetDeltaZ,
                    tanPitch,
                    bracket.lowTimeSec(),
                    bracket.highTimeSec());
            solvedTimeSec = timeResult.timeSec();
            residualMeters = timeResult.residualMeters();
        } else if (Double.isFinite(bracket.bestAbsResidualMeters())
                && bracket.bestAbsResidualMeters() <= (request.verticalErrorToleranceMeters() * 1.5)) {
            solvedTimeSec = bracket.bestTimeSec();
            residualMeters = bracket.bestAbsResidualMeters();
        } else {
            return new SolveResult(
                    SolveStatus.NO_VALID_FLIGHT_TIME,
                    0.0,
                    0.0,
                    new Translation2d(),
                    0.0,
                    Double.POSITIVE_INFINITY);
        }

        Translation2d requiredBallVelocityField = deltaXY.div(solvedTimeSec);
        Translation2d requiredMuzzleVelocityField = requiredBallVelocityField.minus(request.robotVelocityFieldMps());
        double horizontalMuzzleSpeedMps = requiredMuzzleVelocityField.getNorm();
        if (!Double.isFinite(horizontalMuzzleSpeedMps)) {
            return invalidResult();
        }

        Rotation2d yawField = requiredMuzzleVelocityField.getAngle();
        double requiredMuzzleSpeedMps = horizontalMuzzleSpeedMps / cosPitch;

        // Optional first-order drag compensation on required speed magnitude.
        // This correction keeps solver math analytic while allowing one tunable term
        // to offset systematic under-shoot from aerodynamic losses.
        if (request.dragCoefficientPerMeter() > 0.0 && request.dragSpeedCompensationGain() > 0.0) {
            double correction = 1.0 + (request.dragCoefficientPerMeter()
                    * horizontalDistance
                    * request.dragSpeedCompensationGain());
            correction = Math.max(1.0, correction);
            requiredMuzzleSpeedMps *= correction;
            horizontalMuzzleSpeedMps *= correction;
            requiredMuzzleVelocityField = new Translation2d(horizontalMuzzleSpeedMps, yawField);
        }

        return new SolveResult(
                SolveStatus.SOLVED,
                solvedTimeSec,
                yawField.getRadians(),
                requiredMuzzleVelocityField,
                requiredMuzzleSpeedMps,
                residualMeters);
    }

    private static Bracket findBracket(
            SolveRequest request,
            Translation2d deltaXY,
            double targetDeltaZ,
            double tanPitch) {
        double tMin = request.minFlightTimeSec();
        double tMax = request.maxFlightTimeSec();

        Double prevT = null;
        Double prevF = null;
        Double low = null;
        Double high = null;

        double bestT = Double.NaN;
        double bestAbs = Double.POSITIVE_INFINITY;

        for (int i = 0; i <= BRACKET_SAMPLES; i++) {
            double alpha = (double) i / (double) BRACKET_SAMPLES;
            double t = tMin + ((tMax - tMin) * alpha);
            double f = verticalResidual(
                    t,
                    deltaXY,
                    targetDeltaZ,
                    request.robotVelocityFieldMps(),
                    tanPitch,
                    request.gravityMetersPerSec2());

            if (!Double.isFinite(f)) {
                continue;
            }

            double abs = Math.abs(f);
            if (abs < bestAbs) {
                bestAbs = abs;
                bestT = t;
            }

            if (prevT != null && prevF != null && hasOppositeSign(prevF, f)) {
                low = prevT;
                high = t;
                break;
            }
            prevT = t;
            prevF = f;
        }

        return new Bracket(low, high, bestT, bestAbs);
    }

    private static SolveTimeResult solveTimeBisection(
            SolveRequest request,
            Translation2d deltaXY,
            double targetDeltaZ,
            double tanPitch,
            double lowTimeSec,
            double highTimeSec) {
        double low = lowTimeSec;
        double high = highTimeSec;
        double fLow = verticalResidual(
                low,
                deltaXY,
                targetDeltaZ,
                request.robotVelocityFieldMps(),
                tanPitch,
                request.gravityMetersPerSec2());

        double bestT = 0.5 * (low + high);
        double bestAbs = Double.POSITIVE_INFINITY;

        for (int iter = 0; iter < request.maxIterations(); iter++) {
            double mid = 0.5 * (low + high);
            double fMid = verticalResidual(
                    mid,
                    deltaXY,
                    targetDeltaZ,
                    request.robotVelocityFieldMps(),
                    tanPitch,
                    request.gravityMetersPerSec2());
            if (!Double.isFinite(fMid)) {
                break;
            }

            double abs = Math.abs(fMid);
            if (abs < bestAbs) {
                bestAbs = abs;
                bestT = mid;
            }

            if (abs <= request.verticalErrorToleranceMeters()
                    || (high - low) <= request.flightTimeToleranceSec()) {
                return new SolveTimeResult(mid, abs);
            }

            if (hasOppositeSign(fLow, fMid)) {
                high = mid;
            } else {
                low = mid;
                fLow = fMid;
            }
        }

        return new SolveTimeResult(bestT, bestAbs);
    }

    private static double verticalResidual(
            double flightTimeSec,
            Translation2d deltaXY,
            double targetDeltaZ,
            Translation2d robotVelocityFieldMps,
            double tanPitch,
            double gravityMetersPerSec2) {
        if (!Double.isFinite(flightTimeSec) || flightTimeSec <= 1e-9) {
            return Double.NaN;
        }

        Translation2d requiredBallVelocityField = deltaXY.div(flightTimeSec);
        Translation2d requiredMuzzleVelocityField = requiredBallVelocityField.minus(robotVelocityFieldMps);
        double horizontalMuzzleSpeedMps = requiredMuzzleVelocityField.getNorm();
        if (!Double.isFinite(horizontalMuzzleSpeedMps)) {
            return Double.NaN;
        }

        double verticalSpeedMps = horizontalMuzzleSpeedMps * tanPitch;
        return (verticalSpeedMps * flightTimeSec)
                - (0.5 * gravityMetersPerSec2 * flightTimeSec * flightTimeSec)
                - targetDeltaZ;
    }

    private static boolean hasOppositeSign(double a, double b) {
        return (a == 0.0) || (b == 0.0) || ((a > 0.0 && b < 0.0) || (a < 0.0 && b > 0.0));
    }

    private static SolveResult invalidResult() {
        return new SolveResult(
                SolveStatus.INVALID_INPUT,
                0.0,
                0.0,
                new Translation2d(),
                0.0,
                Double.POSITIVE_INFINITY);
    }

    private static boolean isFinitePositive(double value) {
        return Double.isFinite(value) && value > 0.0;
    }

    private record Bracket(
            Double lowTimeSec,
            Double highTimeSec,
            double bestTimeSec,
            double bestAbsResidualMeters) {
        boolean hasSignChange() {
            return lowTimeSec != null && highTimeSec != null;
        }
    }

    private record SolveTimeResult(double timeSec, double residualMeters) {
    }
}
