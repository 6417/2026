package frc.robot;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.fridowpi.motors.utils.FeedForwardValues;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.swerve.ModuleConfig;

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
        public static final String limelightID = "limelight-vier";

        public static final List<Double> aprilTagsForOuttakeStateTeamIsRed = Arrays.asList(17.0, 18.0, 19.0, 20.0, 21.0,
                22.0);
        public static final List<Double> aprilTagsForIntakeStateTeamIsRed = Arrays.asList(1.0, 2.0);

        public static final List<Double> aprilTagsForOuttakeStateTeamIsBlue = Arrays.asList(6.0, 7.0, 8.0, 9.0, 10.0,
                11.0);
        public static final List<Double> aprilTagsForIntakeStateTeamIsBlue = Arrays.asList(12.0, 13.0);
    }

    public static final class SwerveDrive {
        public static ModuleConfig[] configs = new ModuleConfig[4];
        public static boolean isGyroInverted = false;

        public static final double maxSpeed = 4.9; // TODO: for testing
        public static ModuleConfig defaultModuleConfig2024 = new ModuleConfig();
        public static final double moduleXoffset = 0.267;
        public static final double moduleYoffset = 0.267;
        public static final double maxTurnSpeed = 10;// 12// Math.hypot(moduleXoffset, moduleYoffset) * maxSpeed /
                                                     // (Math.PI *
                                                     // 2); // rps

        static {
            defaultModuleConfig2024.maxSpeed = maxSpeed;
            defaultModuleConfig2024.wheelCircumference = Units.inchesToMeters(4) * Math.PI * 0.977 * 1.058376;

            defaultModuleConfig2024.driveGearboxRatio = 6.181;
            defaultModuleConfig2024.driveMotorStallCurrentLimit = 70;
            defaultModuleConfig2024.driveMotorFreeCurrentLimit = 40;
            defaultModuleConfig2024.drivePidValues = new PidValues(1.1926E-05, 0.02, 0);
            // defaultModuleConfig2024.driveFFValues = new FeedForwardValues(0.13271 / 12,
            // 2.1395 / 12, 0.15313 / 12);
            defaultModuleConfig2024.driveFFValues = new FeedForwardValues(0.057, 0.1177, 0.05);

            defaultModuleConfig2024.angleGearboxRatio = 7.44;
            defaultModuleConfig2024.angleMotorStallCurrentLimit = 35;
            defaultModuleConfig2024.angleMotorFreeCurrentLimit = 20;
            defaultModuleConfig2024.angleMotorIzone = 0.1;
            defaultModuleConfig2024.anglePidValues = new PidValues(0.4, 0.01, 0.05);

            defaultModuleConfig2024.encoderThicksToRotationFalcon = 1;
            defaultModuleConfig2024.encoderVelocityToRPSFalcon = 1;
            defaultModuleConfig2024.encoderThicksToRotationNEO = 1;
            defaultModuleConfig2024.encoderVelocityToRPSNEO = 1;

            defaultModuleConfig2024.swerveDebug = true;

            final int LOC_FL = frc.robot.swerve.SwerveDrive.LOC_FL;
            final int LOC_FR = frc.robot.swerve.SwerveDrive.LOC_FR;
            final int LOC_RL = frc.robot.swerve.SwerveDrive.LOC_RL;
            final int LOC_RR = frc.robot.swerve.SwerveDrive.LOC_RR;

            configs[LOC_FL] = defaultModuleConfig2024.clone();
            configs[LOC_FR] = defaultModuleConfig2024.clone();
            configs[LOC_RL] = defaultModuleConfig2024.clone();
            configs[LOC_RR] = defaultModuleConfig2024.clone();

            // Front is on the Batterys side if you use the (Test) Swerve Chassis from 2026
            configs[LOC_FL].driveMotorID = 1;
            configs[LOC_FL].angleMotorID = 11;
            configs[LOC_FL].driveMotorInverted = false;
            configs[LOC_FL].angleMotorInverted = true;
            configs[LOC_FL].moduleOffset = new Translation2d(moduleXoffset, moduleYoffset);
            configs[LOC_FL].encoderChannel = 0;
            configs[LOC_FL].absEncoderOffset = 0.508;

            configs[LOC_FR].driveMotorID = 2;
            configs[LOC_FR].angleMotorID = 12;
            configs[LOC_FR].driveMotorInverted = false;
            configs[LOC_FR].angleMotorInverted = true;
            configs[LOC_FR].moduleOffset = new Translation2d(moduleXoffset, -moduleYoffset);
            configs[LOC_FR].encoderChannel = 1;
            configs[LOC_FR].absEncoderOffset = 0.940;

            configs[LOC_RL].driveMotorID = 3;
            configs[LOC_RL].angleMotorID = 13;
            configs[LOC_RL].driveMotorInverted = false;
            configs[LOC_RL].angleMotorInverted = true;
            configs[LOC_RL].moduleOffset = new Translation2d(-moduleXoffset, moduleYoffset);
            configs[LOC_RL].encoderChannel = 2;
            configs[LOC_RL].absEncoderOffset = 0.056;

            configs[LOC_RR].driveMotorID = 4;
            configs[LOC_RR].angleMotorID = 14;
            configs[LOC_RR].driveMotorInverted = false;
            configs[LOC_RR].angleMotorInverted = true;
            configs[LOC_RR].moduleOffset = new Translation2d(-moduleXoffset, -moduleYoffset);
            configs[LOC_RR].encoderChannel = 3;
            configs[LOC_RR].absEncoderOffset = 0.579;
        }

    }
}
