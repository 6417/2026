package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import frc.fridowpi.motors.utils.FeedForwardValues;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.utils.LinearInterpolationTable;

import static edu.wpi.first.units.Units.Meters;

import java.util.List;
import java.awt.geom.Point2D;
import java.util.Map;
import java.util.Optional;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.fridowpi.motors.FridolinsMotor.IdleMode;

public class Constants {
    // Set to true during tuning sessions; false for competition.
    // When true, shooter RPMs are read live from the dashboard instead of
    // interpolation tables.
    public static final boolean TUNING_MODE = false;

    public static final class Field {
        public static final double FIELD_LENGTH_METERS = 16.540988;
        public static final double FIELD_WIDTH_METERS = 8.069326;
        public static final double FIELD_WIDTH_INCHES = 317.69;
        public static final double FIELD_LENGTH_INCHES = 651.22;

        public static final Pose2d HUB_CENTER_BLUE = new Pose2d(
                Units.inchesToMeters(23.5 + 158.6),
                Units.inchesToMeters(Field.FIELD_WIDTH_INCHES / 2),
                null);

        public static final Pose2d HUB_CENTER_RED = new Pose2d(
                Units.inchesToMeters(Field.FIELD_LENGTH_INCHES - (23.5 + 158.6)), // X= 11.915 meters
                Units.inchesToMeters(Field.FIELD_WIDTH_INCHES / 2), // Y= 4.032 meters
                null);

        public static final double RADIUS_TO_HUB = 3.0; // in meters
        public static final double START_NEUTRALZONE_INCHES = 177.17;

        // Manual fallback pose for odometry reset button.
        // Fill in the real field position where the robot will be placed before
        // pressing the button.
        // Units:
        // x/y in meters, heading in degrees as Rotation2d.
        // to reset, goto right side of the field in team zone, then reset
        public static final Pose2d ODOMETRY_SET_POSE_RED = new Pose2d(
                Field.FIELD_LENGTH_METERS - 0.498,
                7.532,
                Rotation2d.fromDegrees(180.0));

        public static final Pose2d ODOMETRY_SET_POSE_BLUE = new Pose2d(
                0.498,
                0.498,
                Rotation2d.fromDegrees(0.0));

        // to be set in Robot.java based on alliance
        public static Pose2d EDGERight;
        public static Pose2d EDGELeft;
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

    public static final class LEDs {
        public static final int ledPort = 9;
        public static final int ledBufferLength = 12;

        public static final Distance ledsSpacing = Meters.of(0.035);

        // Lower default brightness keeps the strip readable without blinding
        // drivers in the queue or while testing indoors.
        public static final double brightnessScale = 1;

        public static final Map<Double, Color> shootingAnimationKeyframes = Map.of(
            0.0, Color.fromHSV(50, 0, 100),
            0.1, Color.fromHSV(50, 20, 100),
            0.2, Color.fromHSV(50, 30, 100),
            0.3, Color.fromHSV(50, 40, 100),
            0.4, Color.fromHSV(50, 60, 100),
            0.5, Color.fromHSV(50, 80, 100),
            0.6, Color.fromHSV(50, 90, 100),
            0.7, Color.fromHSV(50, 100, 100),
            0.8, Color.fromHSV(50, 100, 100),
            0.9, Color.fromHSV(50, 100, 100)
            );

        public static final Map<Double, Color> intakingAnimationKeyframes = Map.of(
                0.25, Color.fromHSV(60, 50, 100),
                0.5, Color.fromHSV(60, 50, 25),
                0.75, Color.fromHSV(60, 50, 100),
                1.0, Color.fromHSV(60, 50, 25));
    }

    public static final class Limelight {
        public static boolean useVisionUnderTurret = true;
        public static boolean useVisionOnTurret = true;
        public static final String underTurretLimelight = "limelight-undturr";
        public static final String onTurretLimelight = "limelight-onturr";
        public static final Pose3d zeroDegreesTurretLimelightOnTurret = new Pose3d(0.101928, 0.187121, 0.475335,
                new Rotation3d());
        public static final Pose3d turretRotationMiddlePoseToLimelight = new Pose3d(0.10193, 0.02512, 0,
                new Rotation3d());

        public static Vector<N3> standardDevs = VecBuilder.fill(0.2, 0.2, 9999999);
        // Higher base uncertainty for on-turret: turret encoder error and mechanical
        // compliance add position uncertainty beyond pure MegaTag2 tag-distance noise.
        public static Vector<N3> onTurretStdDevs = VecBuilder.fill(0.5, 0.5, 9999999);
        public static int throttleWhileDisabled = 200;
        public static int throttleWhileEnabled = 0;
    }

    public static final class TurretSubsystem { // TODO: set constants
        public static final int ID = 42;

        public static final Translation2d TURRET_OFFSET = new Translation2d(0.162, 0.0); // in meters

        public static final double kMaxVelocity = 1600;
        public static final double kMaxAcceleration = 6000;
        public static final double kAllowedClosedLoopError = 0;

        public static final PidValues pidValuesRotation = new PidValues(0.1, 0.001, 0.06);
        public static final double iZone = 1;
        public static final double iMaxAccum = 100;

        public static final double kConversationRatio = 26.0 / 145.0;
        public static final double kGearRatio = 5.0;

        public static final double resetEncoderPositionDegrees = 112;
        public static final double zeroingVoltage = 1.5;
        public static final double zeroingCurrentThresholdAmps = 27.9; // new turret 7/03/2026
        public static final double zeroingTimeoutSec = 0.5;
        public static final double turretTollerance = 0.4;

        public static final double[] tickRange = { -8.643, 8.81 };

        public static final double pitchMotorForwardLimit = tickRange[1] - 0.2; // for safety measures, leave some
                                                                                // buffer.
        public static final double pitchMotorReverseLimit = tickRange[0] + 0.2;

        public static FeedForwardValues kFeedForward = new FeedForwardValues(0.15, 0, 0);

        public static final int stallCurrentLimit = 30;
        public static final int freeCurrentLimit = 30;
    }

    public static final class SwerveSubsystem {
        public static final double maxSpeed = 4.9; // TODO: for testing
        public static final double moduleXoffset = 0.262;
        public static final double moduleYoffset = 0.262;
        public static final double maxTurnSpeed = 10;// 12// Math.hypot(moduleXoffset, moduleYoffset) * maxSpeed /
                                                     // (Math.PI *
                                                     // 2); // rps
        public static final boolean oldTurnSystem = true;
        public static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.15, 2.2, 0);

    }

    public static final class Controls {
        public static final double deadBandDrive = 0.08;
        public static final double deadBandTurn = 0.08;
    }

    public static final class Intake {
        public static final int intakeMotorId = 10;

        public static final InvertedValue intakeMotorInverted = InvertedValue.CounterClockwise_Positive;

        public static final double intakeSpeed = 0.3;
        public static final double outtakeSpeed = -0.3; // percent

        public static final double intakeSpeedRPM = 3000 / 60;
        public static final double outtakeSpeedRPM = -3000 / 60;

        public static final PidValues pid = new PidValues(0.1, 0.03, 0.00);
        public static final FeedForwardValues ff = new FeedForwardValues(0.15, 0.09);

        // In Ampère
        // current chop: 115
        // stall: 100
        // free: 30
        public static final int stallAmps = 100;
        public static final int freeAmps = 60;

        public static final int currentStuck = 115;

        public static final NeutralModeValue idleMode = NeutralModeValue.Coast;
    }

    public static final class Feeder {
        public static final int motorId = 60;
        public static final boolean motorInverted = false;

        public static PidValues pid = new PidValues(0, 0, 0);
        public static FeedForwardValues ff = new FeedForwardValues(0.27, 0.00225);

        public static final double defaultRPM = 9999; // If this value is 9999 the run() method uses Dutycycle

        public static final IdleMode idleMode = IdleMode.kCoast;

        public static final double pulseForwardDuration = 0.9;
        public static final double pulseReverseDuration = 0.15;
    }

    public static final class Indexer {
        public static int motorID = 31;
        public static IdleMode mode = IdleMode.kCoast;

        public static boolean motorInverted = true;

        public static PidValues pid = new PidValues(0, 0, 0);
        public static FeedForwardValues ff = new FeedForwardValues(0.27, 0.00225);

        public static final int beamBreakSenderDio = 2; // DIO 2 = light sender
        public static final int beamBreakDio = 1; // DIO 1 = light receiver
        public static final boolean beamBreakInverted = false;

        public static final double defaultRPM = 1200;
    }

    public static final class Shooter {
        public static final int topMotorId = 41;
        public static final int bottomMotorId = 40;

        public static final double kP = 0.0001;
        public static final double kI = 0.0;
        public static final double kD = 0.0045;
        public static final double kS_Top = 0.02;
        public static final double kV_Top = 0.001772;
        public static final double kS_Bottom = 0.036;
        public static final double kV_Bottom = 0.0017415;
        public static final double maxRpm = 6000.0;
        public static final boolean bottomMotorInverted = false;
        public static final boolean topMotorInverted = true;

        public static final PidValues pidBoth = new PidValues(kP, kI, kD);
        public static final FeedForwardValues ffTop = new FeedForwardValues(kS_Top, kV_Top);
        public static final FeedForwardValues ffBottom = new FeedForwardValues(kS_Bottom, kV_Bottom);

        public static final double shooterAngle = 60.0;
        public static final double shooterWheelDiameter_meters = 0.055;

        // RPM conversion scale for shoot on move
        // if it shoots too far (near) when driving towards(away from) target, increase
        // if it shoots too near (far) when driving towards(away from) target, decrease
        public static final double rpmConversionFactorScale = 0.7;

        public static final double defaultRPM = 3000;
        public static final double neutralZoneRPM = 2500;

        public static final double motorTolerance = 0;

        // Distance (m) -> RPM tables
        // Measured data points: (distance_meters, rpm)
        // Add more points between/beyond these for a better curve.
        private static final Point2D[] kTopRpmPoints = new Point2D.Double[] {
                new Point2D.Double(2.08, 4000),
                new Point2D.Double(2.61, 2340),
                new Point2D.Double(3.62, 2500),
                new Point2D.Double(4.76, 3200),
        };

        private static final Point2D[] kBottomRpmPoints = new Point2D.Double[] {
                new Point2D.Double(2.08, 600),
                new Point2D.Double(2.61, 2440),
                new Point2D.Double(3.62, 2700),
                new Point2D.Double(4.76, 3200),
        };

        public static final LinearInterpolationTable topRpmTable = new LinearInterpolationTable(kTopRpmPoints);
        public static final LinearInterpolationTable bottomRpmTable = new LinearInterpolationTable(kBottomRpmPoints);

        // Distance (m) -> ball flight time (seconds) — tune from real measurements
        private static final Point2D[] kFlightTimePoints = new Point2D.Double[] {

                // DONE@HOTEL measure these flight times. Note: we want the flight time until
                // the ball enters the TOP of the hub.
                new Point2D.Double(2.77, 0.883),
                new Point2D.Double(3.44, 1.0),
        };
        public static final LinearInterpolationTable flightTimeTable = new LinearInterpolationTable(kFlightTimePoints);

        public static final IdleMode idleMode = IdleMode.kCoast;
    }

    public static final class ShootOnMove {
        public static final double MAX_SHOOT_SPEED_MPS = 1.5;
    }

    public static final class Climber {
        public static final int motorId = 30;
        public static final boolean motorInverted = false; // top Position is highest Value
        public static final IdleMode idleMode = IdleMode.kBrake;

        public static final double resetEncoderPosition = 0.0;
        public static final double homingSpeed = 0.08;
        public static final double homingAmpsThreshold = 1.3;
        public static final double zeroingTimeoutSec = 0.5;
        public static final double zeroingCurrentThreshold = 0.045;

        public static final double climbSpeed = 0.4;
        public static final double prepareClimbSpeed = -0.05;

        public static final PidValues pidValuesOut = new PidValues(0.05, 0.0, 0.6, 0.0);
        public static final PidValues pidValuesIn = new PidValues(0.05, 0.0, 0.2, 0.0);
        public static final Slot1Configs motionMagicSlot1 = new Slot1Configs();
        public static final double kPSlot1 = 1.8;
        public static final double kISlot1 = 0.0;
        public static final double kDSlot1 = 0.0;
        public static final double kSSlot1 = 0.0;
        public static final double kVSlot1 = 0.0;
        public static final double kASlot1 = 0.0;
        public static final double kGSlot1 = -0.185;

        public static final double kPSlot2 = 1.8;
        public static final double kISlot2 = 1.0;
        public static final double kDSlot2 = 0.0;
        public static final double kSSlot2 = 0.0;
        public static final double kVSlot2 = 0.0;
        public static final double kASlot2 = 0.0;
        public static final double kGSlot2 = -0.185;

        public static final double gainSchedErrorThreshold = 0.8;

        public static final Optional<Double> kG = Optional.of(0.0);

        public static double allowedClosedLoopErrorOut = 0.5;
        public static double maxAccelerationOut = 1000; // halfed
        public static double maxVelocityOut = 10; // 3000 led to white sparkles

        public static double allowedClosedLoopErrorIn = 0.5;
        public static double maxAccelerationIn = 60000;
        public static double maxVelocityIn = 6000;

        public static final double climbedPositionDifference = 13;
        public static double highPositionDifference = 27.283;
        public static final double positionTolerance = 0.2;
    }
}
