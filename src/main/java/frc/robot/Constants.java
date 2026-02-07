package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.fridowpi.motors.utils.FeedForwardValues;
import frc.fridowpi.motors.utils.PidValues;

public class Constants {
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
                Units.inchesToMeters(Field.FIELD_LENGTH_INCHES - (23.5 + 158.6)),
                Units.inchesToMeters(Field.FIELD_WIDTH_INCHES / 2),
                null);

        // to be set in Robot.java based on alliance
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
        public static final String driveLimelight = "limelight-drive";
        public static final String limelight2 = "limelight-2";
        public static Vector<N3> standardDevs = VecBuilder.fill(0.3, 0.3, 9999999);
    }

    public static final class TurretSubsystem { //TODO: set constants
        public static final int ID = 999;

        public static final double pitchMotorForwardLimit = 0;
        public static final double pitchMotorReverseLimit = 0;

        public static final Translation2d TURRET_OFFSET = new Translation2d(0.162, 0.0); // in meters

        public static final double kMaxVelocity = 0;
        public static final double kMaxAcceleration = 0;
        public static final double kAllowedClosedLoopError = 0;

        public static final PidValues pidValuesRotation = new PidValues(0, 0, 0);

        public static final int kGearRatio = 0;

        public static final double angularOffset = 0;

        public static FeedForwardValues kFeedForward = new FeedForwardValues(0, 0, 0);
    }

    public static final class SwerveSubsystem {
        public static final double maxSpeed = 4.9; // TODO: for testing
        public static final double moduleXoffset = 0.267;
        public static final double moduleYoffset = 0.267;
        public static final double maxTurnSpeed = 10;// 12// Math.hypot(moduleXoffset, moduleYoffset) * maxSpeed /
                                                     // (Math.PI *
                                                     // 2); // rps
        public static final boolean oldTurnSystem = false;
        public static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.15, 2.2, 0);

    }
    public static final class Controls {
        public static final double deadBandDrive = 0.08;
        public static final double deadBandTurn = 0.08;
    }
}