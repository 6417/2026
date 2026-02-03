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
                                                     // (Math.PI *
                                                     // 2); // rps
        public static final boolean oldTurnSystem = false;
        public static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.15, 2.2, 0);

    }
    public static final class Controls {
        public static final double deadBandDrive = 0.08;
        public static final double deadBandTurn = 0.08;
    }

    public static final class Intake {
        public static final int intakeMotorId = 10;

        public static final boolean intakeMotorInverted = false;

        public static final double intakeSpeed = 0.6;
        public static final double outtakeSpeed = -0.6;

        public static final IdleMode idleMode = IdleMode.kCoast;
    }

    public static final class Shooter {
        public static final int topMotorId = 20;
        public static final int bottomMotorId = 21;

        public static final boolean topMotorInverted = false;
        public static final boolean bottomMotorInverted = true;

        public static final double defaultShootPercent = 0.7;

        // Spin ratio = top / bottom
        public static final double defaultSpinRatio = 1.0;
        public static final double minSpinRatio = 0.6;
        public static final double maxSpinRatio = 1.4;

        // Angle mapping (degrees). This is a simple linear placeholder.
        public static final double minAngleDeg = 15.0;
        public static final double maxAngleDeg = 60.0;

        public static final IdleMode idleMode = IdleMode.kCoast;
    }
}
