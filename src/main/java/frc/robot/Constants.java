package frc.robot;

import java.awt.geom.Point2D;

import org.opencv.features2d.FlannBasedMatcher;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.fridowpi.motors.utils.FeedForwardValues;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.utils.LinearInterpolationTable;

public class Constants {
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
        public static String driveLimelight = "limelight-drive";
    }

    public static final class SwerveSubsystem {
        public static final double maxSpeed = 4.9; // TODO: for testing
        public static final double moduleXoffset = 0.267;
        public static final double moduleYoffset = 0.267;
        public static final double maxTurnSpeed = 10;// 12// Math.hypot(moduleXoffset, moduleYoffset) * maxSpeed /
                                                     // (Math.PI * 2); // rps
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

    public static final class Feeder {
        public static final int motorId = 60;

        public static final boolean motorInverted = false;

        public static final IdleMode idleMode = IdleMode.kCoast;
    }

    public static final class Indexer {
        public static int motorID = 31;
        public static IdleMode mode = IdleMode.kCoast;

        public static boolean motorInverted = true;

        public static PidValues pid = new PidValues(0, 0, 0);
        public static FeedForwardValues ff = new FeedForwardValues(0.27, 0.00225); 

        //TODO: Set correct port for the beamBreak
        public static final int beamBreakDio = 0;
        public static final boolean beamBreakInverted = false;
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

        public static final double motorTolerance = 20;

        // Distance (m) -> RPM tables
        // TODO: Place robot at known distances, tune top/bottom RPMs for best shot,
        // then replace these points and/or add more for better curve fitting.
        private static final Point2D[] kTopRpmPoints = new Point2D.Double[] {
                new Point2D.Double(0.0, 0.0),
                new Point2D.Double(0.0, 0.0)
        };

        private static final Point2D[] kBottomRpmPoints = new Point2D.Double[] {
                new Point2D.Double(0.0, 0.0),
                new Point2D.Double(0.0, 0.0)
        };

        public static final LinearInterpolationTable topRpmTable = new LinearInterpolationTable(kTopRpmPoints);
        public static final LinearInterpolationTable bottomRpmTable = new LinearInterpolationTable(kBottomRpmPoints);

        public static final IdleMode idleMode = IdleMode.kCoast;
    }
}
