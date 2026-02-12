package frc.robot;

import java.awt.geom.Point2D;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.fridowpi.motors.utils.FeedForwardValues;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.utils.LinearInterpolationTable;

public class Constants {
    public static final class Field {
        // 2026 Game Manual section 5.2
        public static final double FIELD_WIDTH_INCHES = 317.7;
        public static final double FIELD_LENGTH_INCHES = 651.2;
        public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(FIELD_LENGTH_INCHES);
        public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(FIELD_WIDTH_INCHES);
        // 2026 Game Manual section 5.4
        public static final double HUB_OUTER_SIZE_INCHES = 47.0;
        public static final double HUB_CENTER_FROM_ALLIANCE_WALL_INCHES = 158.6;

        public static final Pose2d HUB_CENTER_BLUE = new Pose2d(
                Units.inchesToMeters((HUB_OUTER_SIZE_INCHES * 0.5) + HUB_CENTER_FROM_ALLIANCE_WALL_INCHES),
                Units.inchesToMeters(FIELD_WIDTH_INCHES / 2.0),
                new Rotation2d());

        public static final Pose2d HUB_CENTER_RED = new Pose2d(
                Units.inchesToMeters(
                        FIELD_LENGTH_INCHES
                                - ((HUB_OUTER_SIZE_INCHES * 0.5) + HUB_CENTER_FROM_ALLIANCE_WALL_INCHES)),
                Units.inchesToMeters(FIELD_WIDTH_INCHES / 2.0),
                new Rotation2d());

        public static Pose2d EDGE;
        public static Pose2d HUB_CENTER;
        public static double neutralZoneStartX;
    }

    public static final class Joystick {
        public static final int driveJoystickId = 0;
        public static final int operatorJoystickId = 1;
        public static final int idCounterStart = 1000;
        public static final double lt_rt_reshold = 0.2;
    }

    public static final class Gyro {
        public static final int PIGEON_ID = 0;
    }

    public static final class Limelight {
        public static boolean useVision = true;
        public static String driveLimelight = "limelight-drive";
        public static final String limelight2 = "limelight-2";
        public static Vector<N3> standardDevs = VecBuilder.fill(1, 1, 9999999);
    }

    public static final class SwerveSubsystem {
        public static final double maxSpeed = 4.9;
        public static final double moduleXoffset = 0.267;
        public static final double moduleYoffset = 0.267;
        public static final double maxTurnSpeed = 10;
        public static final boolean oldTurnSystem = false;
        public static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.15, 2.2, 0);
    }

    public static final class Controls {
        public static final double deadBandDrive = 0.08;
        public static final double deadBandTurn = 0.08;
    }

    public static final class Intake {
        public static final int intakeMotorId = 10;
        public static final int singulatorMotorId = 11;

        public static final boolean intakeMotorInverted = false;
        public static final boolean singulatorMotorInverted = false;

        public static final double intakeSpeed = 0.6;
        public static final double singulatorSpeed = 0.6;
        public static final double outtakeSpeed = -0.6;
        public static final double outtakeSingulatorSpeed = -0.6;

        public static final IdleMode idleMode = IdleMode.kCoast;
    }

    public static final class TurretSubsystem {
        public static final int ID = 31;

        public static final double pitchMotorForwardLimit = 0;
        public static final double pitchMotorReverseLimit = 0;

        public static final Translation2d TURRET_OFFSET = new Translation2d(0.162, 0.0);

        public static final double kMaxVelocity = 0;
        public static final double kMaxAcceleration = 0;
        public static final double kAllowedClosedLoopError = 0;

        public static final PidValues pidValuesRotation = new PidValues(0, 0, 0);

        public static final int kGearRatio = 1;
        public static final double angularOffset = 0;
        public static final double readyToleranceDeg = 1.0;

        public static final FeedForwardValues kFeedForward = new FeedForwardValues(0, 0, 0);
    }

    public static final class Shooter {
        public static final int topMotorId = 20;
        public static final int bottomMotorId = 21;

        public static final boolean topMotorInverted = true;
        public static final boolean bottomMotorInverted = false;

        public static final double kP = 0.00007;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0001575;
        public static final double maxRpm = 0.0;
        public static final double maxNegPower = -0.30;

        public static final double defaultTopRpm = 3000.0;
        public static final double defaultBottomRpm = 2800.0;
        public static final double motorTolerance = 20;
        // Shooter wheel geometry and effective transfer efficiency:
        // ideal: v_ball = (2*pi/60) * wheelRadius * avgWheelRpm
        // real:  v_ball = eta * ideal, with eta accounting for slip/compression/losses.
        public static final double wheelRadiusMeters = 0.025;
        public static final double shooterSlipEfficiencyEta = 0.8492;
        // Keep this as the single conversion constant used by the code paths.
        public static final double rpmToMpsFactor =
                shooterSlipEfficiencyEta * ((2.0 * Math.PI / 60.0) * wheelRadiusMeters);

        public static final double movingShotScaleMin = 0.85;
        public static final double movingShotScaleMax = 1.15;
        public static final double minFlightTimeSec = 0.10;
        public static final boolean aimAtHubOnly = true;
        public static final boolean enableMovingShotCompensation = true;

        public static final boolean enableFireGate = true;
        public static final double maxRobotOmegaRadPerSecForShot = Math.toRadians(120.0);
        public static final double minShotDistanceMeters = 1.2;
        public static final double maxShotDistanceMeters = 7.0;

        // Set to the active alliance hub in Robot init (defaults to blue before alliance is known).
        public static Translation2d hubPositionField = Field.HUB_CENTER_BLUE.getTranslation();
        // Start with turret center offset and refine from CAD measurements.
        public static final Translation2d shooterOffsetRobot = TurretSubsystem.TURRET_OFFSET;
        public static final double turretZeroOnRobotRad = 0.0;

        private static final Point2D[] kTopRpmPoints = new Point2D.Double[] {
                new Point2D.Double(1.5, 5362.5),
                new Point2D.Double(2.5, 5429.4),
                new Point2D.Double(3.5, 5519.4),
                new Point2D.Double(4.5, 5764.7),
                new Point2D.Double(5.5, 6300.7)
        };

        private static final Point2D[] kBottomRpmPoints = new Point2D.Double[] {
                new Point2D.Double(1.5, 5162.5),
                new Point2D.Double(2.5, 5229.4),
                new Point2D.Double(3.5, 5219.4),
                new Point2D.Double(4.5, 5414.7),
                new Point2D.Double(5.5, 5900.7)
        };

        private static final Point2D[] kFlightTimePoints = new Point2D.Double[] {
                new Point2D.Double(1.5, 0.368),
                new Point2D.Double(2.5, 0.407),
                new Point2D.Double(3.5, 0.609),
                new Point2D.Double(4.5, 0.624),
                new Point2D.Double(5.5, 0.650)
        };

        public static final LinearInterpolationTable topRpmTable = new LinearInterpolationTable(kTopRpmPoints);
        public static final LinearInterpolationTable bottomRpmTable = new LinearInterpolationTable(kBottomRpmPoints);
        public static final LinearInterpolationTable flightTimeTable = new LinearInterpolationTable(kFlightTimePoints);

        public static final IdleMode idleMode = IdleMode.kCoast;
    }

    public static final class ShooterSim {
        public static final double flywheelGearing = 1.0;
        public static final double topWheelMoiKgM2 = 0.0015;
        public static final double bottomWheelMoiKgM2 = 0.0015;

        public static final double turretKp = 6.0;
        public static final double turretMaxVelocityRadPerSec = Math.toRadians(360.0);

        public static final double launchHeightMeters = 0.70;
        public static final double launchPitchRad = Math.toRadians(35.0);
        public static final double gravityMetersPerSec2 = 9.81;
        // Quadratic drag coefficient k in a_drag = -k * |v| * v (units: 1/m).
        public static final double dragCoefficientPerMeter = 0.09598;
        public static final double maxFlightTimeSec = 2.0;

        // 2026 Game Manual section 5.4: front edge of top opening is 72 in off carpet.
        public static final double hubCenterHeightMeters = Units.inchesToMeters(72.0);
        // 2026 Game Manual section 5.4: top opening is 41.7 in across (hex), approximated as circular radius.
        public static final double hubRadiusMeters = Units.inchesToMeters(41.7 / 2.0);
        public static final double hubHeightToleranceMeters = 0.15;

        public static final double mechanismSizeMeters = 2.0;
        public static final double turretDisplayLengthMeters = 0.8;

        public static final Translation2d scenarioStartPositionField = new Translation2d(2.0, 3.0);
        public static final double scenarioStartHeadingRad = 0.0;
        public static final Translation2d scenarioVelocityFieldMps = new Translation2d(1.0, 0.0);
        public static final double scenarioShotIntervalSec = 0.60;
        public static final double manualTestTopRpm = 3600.0;
        public static final double manualTestBottomRpm = 3400.0;
        public static final double manualTestSpinupSec = 0.60;
        public static final double manualTestMuzzleSpeedMps = 12.0;
        public static final boolean enableAutoDebugShotsInSim = true;
        public static final double autoDebugShotIntervalSec = 10.0;
        public static final boolean autoDebugShotsRequireReady = true;

        public static final int field2dMaxTraceCount = 12;
        public static final int field2dMaxPointsPerTrace = 120;
    }
}
