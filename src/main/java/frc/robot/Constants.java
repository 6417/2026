package frc.robot;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.fridowpi.motors.utils.FeedForwardValues;
import frc.fridowpi.motors.utils.PidValues;

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
        // TODO: Verify these IDs/inversions on the real robot.
        public static final int singulatorMotorId = 20;
        public static final int feederMotorId = 21;

        public static final boolean singulatorMotorInverted = false;
        public static final boolean feederMotorInverted = false;

        public static final double singulatorSpeed = 0.6;
        public static final double feederSpeed = 0.6;
        public static final double outtakeSingulatorSpeed = -0.6;
        public static final double outtakeFeederSpeed = -0.6;

        // Beam break on a roboRIO DIO (Digital I/O) port.
        // Set beamBreakInverted true if the sensor is active-high (true when beam is broken).
        public static final int beamBreakDio = 0;
        public static final boolean beamBreakInverted = false;

        // TODO: Tune feeder velocity PID (Spark Max velocity uses RPM).
        public static final PidValues feederVelocityPid = new PidValues(0.0, 0.0, 0.0);

        public static final IdleMode idleMode = IdleMode.kCoast;
    }
}
