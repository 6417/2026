package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.fridowpi.motors.utils.FeedForwardValues;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.utils.LinearInterpolationTable;

import java.awt.geom.Point2D;
import java.util.Optional;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;

import frc.fridowpi.motors.FridolinsMotor.IdleMode;

public class Constants {
    // Set to true during tuning sessions; false for competition.
    // When true, shooter RPMs are read live from the dashboard instead of interpolation tables.
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
        public static final double START_NEUTRALZONE_INCHES = 165;

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

    public static final class Limelight {
        public static boolean useVisionUnderTurret = true;
        public static boolean useVisionOnTurret= true;
        public static final String underTurretLimelight = "limelight-undturr";
        public static final String onTurretLimelight = "limelight-onturr";
        public static final Pose3d zeroDegreesTurretLimelightOnTurret = new Pose3d(0.101928, 0.187121, 0.475335,  new Rotation3d());
        public static final Pose3d turretRotationMiddlePoseToLimelight = new Pose3d(0.10193, 0.02512, 0, new Rotation3d());

        public static Vector<N3> standardDevs = VecBuilder.fill(0.3, 0.3, 9999999);
        // Higher base uncertainty for on-turret: turret encoder error and mechanical
        // compliance add position uncertainty beyond pure MegaTag2 tag-distance noise.
        public static Vector<N3> onTurretStdDevs = VecBuilder.fill(0.5, 0.5, 9999999);
    }

    public static final class TurretSubsystem { //TODO: set constants
        public static final int ID = 42;

        public static final Translation2d TURRET_OFFSET = new Translation2d(0.162, 0.0); // in meters

        public static final double kMaxVelocity = 1600;
        public static final double kMaxAcceleration = 6000;
        public static final double kAllowedClosedLoopError = 0;

        public static final PidValues pidValuesRotation = new PidValues(0.3, 0.001, 0.03);
        public static final double iZone = 1;
        public static final double iMaxAccum = 100;

        public static final double kConversationRatio = 26.0/145.0;
        public static final double kGearRatio = 5.0;

        public static final double resetEncoderPositionDegrees = 112;
        public static final double zeroingVoltage = 1;
        public static final double zeroingCurrentThresholdAmps = 25; // test
        public static final double zeroingTimeoutSec = 0.5;
        public static final double turretTollerance = 0.1;
 
        public static final double[] tickRange = {-8.643, 8.81};

        public static final double pitchMotorForwardLimit = tickRange[1] - 0.2; // for safety measures, leave some buffer.
        public static final double pitchMotorReverseLimit = tickRange[0] + 0.2;

        public static FeedForwardValues kFeedForward = new FeedForwardValues(0.1, 0, 0);
        
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

        public static final boolean intakeMotorInverted = true;

        public static final double intakeSpeed = 0.3;
        public static final double outtakeSpeed = -0.3; // percent

        public static final double intakeSpeedRPM = 1500;
        public static final double outtakeSpeedRPM = -1500;

        public static final PidValues pid = new PidValues(0.00009, 0, 0.0002);
        public static final FeedForwardValues ff = new FeedForwardValues(0.15, 0.0018);

        // current chop: 115
        // stall: 100
        // free: 30
        public static final int stallAmps = 100;
        public static final int freeAmps = 30;
        public static final int currentChopAmps = 115;

        public static final IdleMode idleMode = IdleMode.kCoast;
    }

    public static final class Feeder {
        public static final int motorId = 60;
        public static final boolean motorInverted = true;

        public static PidValues pid = new PidValues(0, 0, 0);
        public static FeedForwardValues ff = new FeedForwardValues(0.27, 0.00225); 

        public static final double defaultRPM = 3000;

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
        public static final int beamBreakDio = 1;       // DIO 1 = light receiver
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

            // TODO measure these flight times. Note: we want the flight time until the ball enters the TOP of the hub.
                new Point2D.Double(2.0, 0.20),
                new Point2D.Double(3.0, 0.30),
                new Point2D.Double(4.0, 0.40),
                new Point2D.Double(5.0, 0.50),
        };
        public static final LinearInterpolationTable flightTimeTable = new LinearInterpolationTable(kFlightTimePoints);

        public static final IdleMode idleMode = IdleMode.kCoast;
    }

    public static final class ShootOnMove {
        public static final double MAX_SHOOT_SPEED_MPS = 1.5;
        public static final double MAX_VIRTUAL_OFFSET_METERS = 1.0;
    }

    public static final class Climber {
        public static final int motorId = 30;
        public static final boolean motorInverted = false; //top Position is highest Value
        public static final IdleMode idleMode = IdleMode.kBrake;

        public static final double resetEncoderPosition = 0.0;
        public static final double homingSpeed = 0.08;
        public static final double homingAmpsThreshold = 1.3;
        public static final double zeroingTimeoutSec = 0.5;
        public static final double zeroingCurrentThreshold = 0.045;

        public static final double climbSpeed = 0.2;
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
        public static double maxAccelerationOut = 30000;
        public static double maxVelocityOut = 3000;

        public static double allowedClosedLoopErrorIn = 0.5;
        public static double maxAccelerationIn = 60000;
        public static double maxVelocityIn = 6000;

        public static final double climbedPosition = -15;
        public static final double highPosition = -27.5;
        public static final double positionTolerance = 0.2;
    }
}
