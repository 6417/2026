package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.FridolinsMotor;
import frc.robot.Constants;
import frc.robot.sim.CompletedShotTrace;
import frc.robot.sim.ShooterTurretSimulator;
import frc.robot.sim.ShotSample;
import frc.robot.sim.ShotResult;

public class ShooterSubsystem extends SubsystemBase {
    public enum ShotBlockReason {
        NONE,
        DISTANCE_OUT_OF_RANGE,
        ROBOT_TURNING_TOO_FAST,
        SHOOTER_NOT_READY,
        TURRET_NOT_READY
    }

    /**
     * Full moving-shot command calculated from robot state.
     *
     * turretYawRad: turret target angle relative to robot frame (radians).
     * topRpm/bottomRpm: wheel RPM targets to achieve the required ball speed and spin.
     * flightTimeSec: estimated flight time used for lead compensation.
     */
    public record ShotCommand(double turretYawRad, double topRpm, double bottomRpm, double flightTimeSec) {
    }

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
    private double topPercentCommand = 0.0;
    private double bottomPercentCommand = 0.0;
    private double turretTargetAngleRad = 0.0;

    private Pose2d simRobotPoseField = new Pose2d();
    private Translation2d simRobotVelocityFieldMps = new Translation2d();

    private final boolean simulationEnabled;
    private final ShooterTurretSimulator shooterTurretSimulator;
    private final Mechanism2d shooterMechanism2d;
    private final MechanismLigament2d turretLigament;
    private Optional<ShotResult> latestSimShotResult = Optional.empty();
    private Optional<ShotSample> latestActiveShotSample = Optional.empty();
    private final List<CompletedShotTrace> pendingCompletedShotTraces = new ArrayList<>();
    private int simShotsFired = 0;
    private double lastSimFireTimestampSec = -1.0;
    private double lastSimFireMuzzleSpeedMps = 0.0;
    private double lastTargetDistanceMeters = Double.NaN;
    private ShotBlockReason lastShotBlockReason = ShotBlockReason.NONE;

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

        simulationEnabled = RobotBase.isSimulation();
        if (simulationEnabled) {
            shooterTurretSimulator = new ShooterTurretSimulator(
                    Constants.ShooterSim.flywheelGearing,
                    Constants.ShooterSim.topWheelMoiKgM2,
                    Constants.ShooterSim.bottomWheelMoiKgM2,
                    Constants.ShooterSim.turretKp,
                    Constants.ShooterSim.turretMaxVelocityRadPerSec,
                    Constants.ShooterSim.launchHeightMeters,
                    Constants.ShooterSim.launchPitchRad,
                    Constants.Shooter.hubPositionField,
                    Constants.ShooterSim.hubCenterHeightMeters,
                    Constants.ShooterSim.hubRadiusMeters,
                    Constants.ShooterSim.hubHeightToleranceMeters,
                    Constants.ShooterSim.gravityMetersPerSec2,
                    Constants.ShooterSim.maxFlightTimeSec);

            shooterMechanism2d = new Mechanism2d(
                    Constants.ShooterSim.mechanismSizeMeters,
                    Constants.ShooterSim.mechanismSizeMeters);
            MechanismRoot2d root = shooterMechanism2d.getRoot(
                    "turretRoot",
                    Constants.ShooterSim.mechanismSizeMeters * 0.5,
                    Constants.ShooterSim.mechanismSizeMeters * 0.5);
            turretLigament = new MechanismLigament2d(
                    "turret",
                    Constants.ShooterSim.turretDisplayLengthMeters,
                    0.0,
                    6.0,
                    new Color8Bit(Color.kYellow));
            root.append(turretLigament);
            SmartDashboard.putData("Shooter/Mechanism", shooterMechanism2d);
        } else {
            shooterTurretSimulator = null;
            shooterMechanism2d = null;
            turretLigament = null;
        }
    }

    public void stop() {
        topMotor.stopMotor();
        bottomMotor.stopMotor();
        topPercentCommand = 0.0;
        bottomPercentCommand = 0.0;
    }

    public void setPercent(double topPercent, double bottomPercent) {
        // Open-loop control (no PID). Useful for quick tests.
        double top = MathUtil.clamp(topPercent, -1.0, 1.0);
        double bottom = MathUtil.clamp(bottomPercent, -1.0, 1.0);
        topPercentCommand = top;
        bottomPercentCommand = bottom;
        topMotor.set(top);
        bottomMotor.set(bottom);
    }

    /**
     * Run the shooter using closed-loop RPM control.
     *
     * This sets the same RPM for both wheels and then:
     * 1) clamps the target RPM (if a max is configured),
     * 2) computes PID + feedforward output,
     * 3) sends the final percent output to the motors.
    */
    public void run(double topRpm, double bottomRpm) {
        // Clamp target RPMs if a max RPM is configured.
        targetTopRpm = clampRpm(topRpm);
        targetBottomRpm = clampRpm(bottomRpm);

        // Spark Max encoder velocity is RPM by default.
        double topOutput = calculateOutput(topPid, getCurrentTopRpm(), targetTopRpm);
        double bottomOutput = calculateOutput(bottomPid, getCurrentBottomRpm(), targetBottomRpm);

        // PID + FF output is still a percent (-1..1) to send to the motor.
        topPercentCommand = topOutput;
        bottomPercentCommand = bottomOutput;
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

    /**
     * Calculate a full moving-shot command from robot field pose and robot translational velocity.
     *
     * This method combines:
     * 1) data-driven shooter tuning (distance -> top/bottom RPM + flight time), and
     * 2) vector lead compensation for robot motion while the ball is in flight.
     */
    public ShotCommand calculateShotCommand(Pose2d robotPoseField, Translation2d robotVelocityFieldMps) {
        return calculateShotCommand(
                robotPoseField,
                robotVelocityFieldMps,
                Constants.Shooter.hubPositionField,
                Constants.Shooter.shooterOffsetRobot,
                Constants.Shooter.turretZeroOnRobotRad);
    }

    /**
     * Same moving-shot calculation, but with explicit inputs for easier testing/simulation.
     */
    public ShotCommand calculateShotCommand(
            Pose2d robotPoseField,
            Translation2d robotVelocityFieldMps,
            Translation2d hubPositionField,
            Translation2d shooterOffsetRobot,
            double turretZeroOnRobotRad) {
        // Shooter muzzle position in field frame: robot position + rotated shooter offset.
        Translation2d shooterPositionField =
                robotPoseField.getTranslation().plus(shooterOffsetRobot.rotateBy(robotPoseField.getRotation()));

        // Field-space vector from shooter to hub.
        Translation2d shooterToHubField = hubPositionField.minus(shooterPositionField);
        double distanceMeters = shooterToHubField.getNorm();

        // If target vector is too small, keep default settings to avoid unstable math.
        if (distanceMeters < 1e-6) {
            return new ShotCommand(0.0, Constants.Shooter.defaultTopRpm, Constants.Shooter.defaultBottomRpm,
                    Constants.Shooter.minFlightTimeSec);
        }

        // Stationary baseline from measured tables.
        double baseTopRpm = Constants.Shooter.topRpmTable.getOutput(distanceMeters);
        double baseBottomRpm = Constants.Shooter.bottomRpmTable.getOutput(distanceMeters);
        double flightTimeSec = Math.max(Constants.Shooter.minFlightTimeSec,
                Constants.Shooter.flightTimeTable.getOutput(distanceMeters));

        // Required ball velocity in field frame to reach hub in estimated flight time.
        Translation2d requiredBallVelocityField = shooterToHubField.div(flightTimeSec);

        // Relative muzzle velocity that shooter must create in robot frame context.
        Translation2d requiredMuzzleVelocityField = requiredBallVelocityField.minus(robotVelocityFieldMps);

        // Turret heading setpoint in robot frame.
        double turretYawFieldRad = requiredMuzzleVelocityField.getAngle().getRadians();
        double turretYawRobotRad = MathUtil.angleModulus(
                turretYawFieldRad - robotPoseField.getRotation().getRadians() - turretZeroOnRobotRad);

        // Scale baseline RPM up/down so average wheel speed matches required muzzle speed magnitude.
        double requiredMuzzleSpeedMps = requiredMuzzleVelocityField.getNorm();
        double nominalMuzzleSpeedMps = ballSpeedFromRpm((baseTopRpm + baseBottomRpm) / 2.0);

        double scale = 1.0;
        if (nominalMuzzleSpeedMps > 1e-6) {
            scale = requiredMuzzleSpeedMps / nominalMuzzleSpeedMps;
            scale = MathUtil.clamp(scale, Constants.Shooter.movingShotScaleMin, Constants.Shooter.movingShotScaleMax);
        }

        double correctedTopRpm = clampRpm(baseTopRpm * scale);
        double correctedBottomRpm = clampRpm(baseBottomRpm * scale);

        return new ShotCommand(turretYawRobotRad, correctedTopRpm, correctedBottomRpm, flightTimeSec);
    }

    /**
     * Convenience API: calculate moving-shot targets and apply the wheel RPM outputs.
     */
    public ShotCommand shootWhileMoving(Pose2d robotPoseField, Translation2d robotVelocityFieldMps) {
        ShotCommand command = calculateShotCommand(robotPoseField, robotVelocityFieldMps);
        applyShotCommand(command);
        return command;
    }

    /**
     * Apply a precomputed shot command to turret target + shooter wheels.
     */
    public void applyShotCommand(ShotCommand command) {
        setTurretTargetAngleRad(command.turretYawRad());
        run(command.topRpm(), command.bottomRpm());
    }

    /**
     * Compute and apply the shot command from current robot pose/velocity.
     */
    public ShotCommand applyShotCommandFromRobotState(Pose2d robotPoseField, Translation2d robotVelocityFieldMps) {
        lastTargetDistanceMeters = computeDistanceToHubMeters(robotPoseField);
        ShotCommand command;
        if (Constants.Shooter.enableMovingShotCompensation) {
            command = calculateShotCommand(robotPoseField, robotVelocityFieldMps);
        } else {
            command = calculateShotCommand(robotPoseField, new Translation2d());
        }
        applyShotCommand(command);
        return command;
    }

    public boolean isShooterReady() {
        return Math.abs(targetBottomRpm - getCurrentBottomRpm()) <= Constants.Shooter.motorTolerance
                && Math.abs(targetTopRpm - getCurrentTopRpm()) <= Constants.Shooter.motorTolerance;
    }

    public double getLastTargetDistanceMeters() {
        return lastTargetDistanceMeters;
    }

    public ShotBlockReason getLastShotBlockReason() {
        return lastShotBlockReason;
    }

    public boolean isDistanceInRange() {
        if (Double.isNaN(lastTargetDistanceMeters)) {
            return false;
        }
        return lastTargetDistanceMeters >= Constants.Shooter.minShotDistanceMeters
                && lastTargetDistanceMeters <= Constants.Shooter.maxShotDistanceMeters;
    }

    public boolean isShotReady(boolean turretReady, double robotOmegaRadPerSec) {
        if (!Constants.Shooter.enableFireGate) {
            lastShotBlockReason = ShotBlockReason.NONE;
            return true;
        }
        if (!isDistanceInRange()) {
            lastShotBlockReason = ShotBlockReason.DISTANCE_OUT_OF_RANGE;
            return false;
        }
        if (Math.abs(robotOmegaRadPerSec) > Constants.Shooter.maxRobotOmegaRadPerSecForShot) {
            lastShotBlockReason = ShotBlockReason.ROBOT_TURNING_TOO_FAST;
            return false;
        }
        if (!isShooterReady()) {
            lastShotBlockReason = ShotBlockReason.SHOOTER_NOT_READY;
            return false;
        }
        if (!turretReady) {
            lastShotBlockReason = ShotBlockReason.TURRET_NOT_READY;
            return false;
        }
        lastShotBlockReason = ShotBlockReason.NONE;
        return true;
    }

    /**
     * Fire only when all shot gates are satisfied.
     */
    public boolean tryFire(boolean turretReady, double robotOmegaRadPerSec) {
        if (!isShotReady(turretReady, robotOmegaRadPerSec)) {
            return false;
        }
        simulateFire();
        return true;
    }

    private double clampRpm(double rpm) {
        /**
         * Clamp target RPM to a safe range.
         *
         * If maxRpm is set to 0, we treat it as "no limit" and return the input.
         * This avoids accidentally limiting the shooter when maxRpm is not configured yet.
         */
        if (Constants.Shooter.maxRpm > 0.0) {
            rpm = MathUtil.clamp(rpm, -Constants.Shooter.maxRpm, Constants.Shooter.maxRpm);
        }
        return rpm;
    }

    private double calculateOutput(PIDController pid, double currentRpm, double targetRpm) {
        /**
         * Compute motor output from RPM error.
         *
         * PID handles error correction, feedforward provides the base output
         * needed to maintain the target RPM. The result is clamped to a safe
         * output range before sending to the motor.
         */
        // PID does the error correction, FF gives a baseline voltage for the target speed.
        double outputPid = pid.calculate(currentRpm, targetRpm);
        double outputFf = feedforward.calculate(targetRpm);
        double output = outputPid + outputFf;


        // Final safety clamp to motor input range.
        return MathUtil.clamp(output, -1.0, 1.0);
    }

    public void setTurretTargetAngleRad(double targetAngleRad) {
        turretTargetAngleRad = MathUtil.angleModulus(targetAngleRad);
    }

    public double getTurretAngleRad() {
        if (simulationEnabled) {
            return shooterTurretSimulator.getTurretAngleRad();
        }
        return turretTargetAngleRad;
    }

    public double getTargetTopRpm() {
        return targetTopRpm;
    }

    public double getTargetBottomRpm() {
        return targetBottomRpm;
    }

    public double getCurrentTopRpm() {
        if (simulationEnabled) {
            return shooterTurretSimulator.getTopWheelRpm();
        }
        return topMotor.getEncoderVelocity();
    }

    public double getCurrentBottomRpm() {
        if (simulationEnabled) {
            return shooterTurretSimulator.getBottomWheelRpm();
        }
        return bottomMotor.getEncoderVelocity();
    }

    public void setSimRobotState(Pose2d robotPoseField, Translation2d robotVelocityFieldMps) {
        simRobotPoseField = robotPoseField;
        simRobotVelocityFieldMps = robotVelocityFieldMps;
        lastTargetDistanceMeters = computeDistanceToHubMeters(robotPoseField);
    }

    public void simulateFire() {
        if (!simulationEnabled) {
            return;
        }
        lastSimFireTimestampSec = Timer.getFPGATimestamp();
        lastSimFireMuzzleSpeedMps = Math.max(
                0.0,
                ((getCurrentTopRpm() + getCurrentBottomRpm()) * 0.5) * Constants.Shooter.rpmToMpsFactor);
        simShotsFired++;
        shooterTurretSimulator.fireShot(
                lastSimFireTimestampSec,
                simRobotPoseField,
                simRobotVelocityFieldMps,
                Constants.Shooter.shooterOffsetRobot,
                Constants.Shooter.rpmToMpsFactor);
    }

    public void simulateFireDebug(double muzzleSpeedMps) {
        if (!simulationEnabled) {
            return;
        }
        lastSimFireTimestampSec = Timer.getFPGATimestamp();
        lastSimFireMuzzleSpeedMps = Math.max(0.0, muzzleSpeedMps);
        simShotsFired++;
        shooterTurretSimulator.fireShotWithMuzzleSpeed(
                lastSimFireTimestampSec,
                simRobotPoseField,
                simRobotVelocityFieldMps,
                Constants.Shooter.shooterOffsetRobot,
                Math.max(0.0, muzzleSpeedMps));
    }

    public Optional<ShotResult> getLatestSimShotResult() {
        return latestSimShotResult;
    }

    public Optional<ShotSample> getLatestActiveShotSample() {
        return latestActiveShotSample;
    }

    public List<CompletedShotTrace> drainCompletedShotTraces() {
        List<CompletedShotTrace> drained = new ArrayList<>(pendingCompletedShotTraces);
        pendingCompletedShotTraces.clear();
        return drained;
    }

    public Translation2d getCurrentMuzzlePositionField() {
        return simRobotPoseField.getTranslation().plus(
                Constants.Shooter.shooterOffsetRobot.rotateBy(simRobotPoseField.getRotation()));
    }

    public Rotation2d getCurrentTurretYawField() {
        return Rotation2d.fromRadians(simRobotPoseField.getRotation().getRadians() + getTurretAngleRad());
    }

    public double getSimHitRate() {
        if (!simulationEnabled) {
            return 0.0;
        }
        return shooterTurretSimulator.getHitRate();
    }

    private double ballSpeedFromRpm(double averageWheelRpm) {
        // Conversion from average wheel RPM to approximate ball linear speed (m/s).
        // This factor is empirical and should be tuned from measured shot data.
        return averageWheelRpm * Constants.Shooter.rpmToMpsFactor;
    }

    private double computeDistanceToHubMeters(Pose2d robotPoseField) {
        Translation2d shooterPositionField = robotPoseField.getTranslation().plus(
                Constants.Shooter.shooterOffsetRobot.rotateBy(robotPoseField.getRotation()));
        return Constants.Shooter.hubPositionField.minus(shooterPositionField).getNorm();
    }

    @Override
    public void simulationPeriodic() {
        if (!simulationEnabled) {
            return;
        }

        shooterTurretSimulator.update(
                0.020,
                RobotController.getBatteryVoltage(),
                topPercentCommand,
                bottomPercentCommand,
                turretTargetAngleRad);

        latestSimShotResult = shooterTurretSimulator.getLatestShotResult();
        latestActiveShotSample = shooterTurretSimulator.getLatestActiveShotSample();
        pendingCompletedShotTraces.addAll(shooterTurretSimulator.drainCompletedShotTraces());
        turretLigament.setAngle(Math.toDegrees(shooterTurretSimulator.getTurretAngleRad()));

        SmartDashboard.putNumber("Shooter/TargetTopRpm", targetTopRpm);
        SmartDashboard.putNumber("Shooter/TargetBottomRpm", targetBottomRpm);
        SmartDashboard.putNumber("Shooter/CurrentTopRpm", shooterTurretSimulator.getTopWheelRpm());
        SmartDashboard.putNumber("Shooter/CurrentBottomRpm", shooterTurretSimulator.getBottomWheelRpm());
        SmartDashboard.putNumber("Shooter/TurretTargetDeg", Math.toDegrees(turretTargetAngleRad));
        SmartDashboard.putNumber("Shooter/TurretCurrentDeg", Math.toDegrees(shooterTurretSimulator.getTurretAngleRad()));
        SmartDashboard.putNumber("Shooter/TurretYawFieldDeg", getCurrentTurretYawField().getDegrees());
        SmartDashboard.putNumber("Shooter/SimHitRate", shooterTurretSimulator.getHitRate());
        SmartDashboard.putNumber("Shooter/PendingShotTraces", pendingCompletedShotTraces.size());
        SmartDashboard.putNumber("Shooter/TargetDistanceM", lastTargetDistanceMeters);
        SmartDashboard.putBoolean("Shooter/DistanceInRange", isDistanceInRange());
        SmartDashboard.putString("Shooter/ShotBlockReason", lastShotBlockReason.name());
        SmartDashboard.putBoolean("Shooter/ShooterReady", isShooterReady());
        SmartDashboard.putNumber("Shooter/SimShotsFired", simShotsFired);
        SmartDashboard.putNumber("Shooter/LastSimFireTimestampSec", lastSimFireTimestampSec);
        SmartDashboard.putNumber("Shooter/LastSimFireMuzzleSpeedMps", lastSimFireMuzzleSpeedMps);
        SmartDashboard.putNumber(
                "Shooter/LatestShotTracePoints",
                pendingCompletedShotTraces.isEmpty()
                        ? 0
                        : pendingCompletedShotTraces.get(pendingCompletedShotTraces.size() - 1).samples().size());

        if (latestSimShotResult.isPresent()) {
            ShotResult result = latestSimShotResult.get();
            SmartDashboard.putBoolean("Shooter/LastShotHit", result.hit());
            SmartDashboard.putNumber("Shooter/LastShotClosestDistanceM", result.closestDistanceMeters());
            SmartDashboard.putNumber("Shooter/LastShotHorizontalErrorM", result.horizontalErrorMeters());
            SmartDashboard.putNumber("Shooter/LastShotVerticalErrorM", result.verticalErrorMeters());
            SmartDashboard.putNumber("Shooter/LastShotFlightTimeS", result.flightTimeSec());
        }

        if (latestActiveShotSample.isPresent()) {
            SmartDashboard.putNumber("Shooter/ActiveBallZ", latestActiveShotSample.get().positionField().getZ());
        }
    }
}
