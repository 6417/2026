package frc.robot;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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
    }

    public static final class SwerveSubsystem {
        public static final double maxSpeed = 4.9; // TODO: for testing
        public static final double moduleXoffset = 0.267;
        public static final double moduleYoffset = 0.267;
        public static final double maxTurnSpeed = 10;// 12// Math.hypot(moduleXoffset, moduleYoffset) * maxSpeed /
                                                     // (Math.PI *
                                                     // 2); // rps

        /*

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
            
         
            configs[LOC_FL].absEncoderOffset = 0.508;

          
            configs[LOC_FR].absEncoderOffset = 0.940;

            configs[LOC_RL].absEncoderOffset = 0.056;

            configs[LOC_RR].absEncoderOffset = 0.579;
        }

    }*/
    }
}
