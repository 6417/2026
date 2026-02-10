package frc.robot;

import java.util.Optional;

import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.fridowpi.motors.utils.PidValues;

public class Constants {
    public static final class Joystick {
        public static final int driveJoystickId = 0;
        public static final int operatorJoystickId = 1;
    }

    public static final class Climber {
        // Hardware IDs and motor direction for the climber.
        public static final int motorId = 30;
        public static final boolean motorInverted = false;
        public static final IdleMode idleMode = IdleMode.kBrake;

        // Encoder zeroing parameters (used by ClimberEncoderZero).
        public static final double resetEncoderPosition = 0.0;
        public static final double zeroingSpeed = -0.1;
        public static final double zeroingTimeoutSec = 0.5;
        public static final double zeroingCurrentThreshold = 0.045;

        // PID slots for Motion Magic (TalonFX/Kraken):
        // slot 0 = extend (out), slot 1 = retract (in)
        public static final PidValues pidValuesOut = new PidValues(0.05, 0.0, 0.6, 0.0);
        public static final PidValues pidValuesIn = new PidValues(0.05, 0.0, 0.2, 0.0);
        public static final Optional<Double> kG = Optional.of(0.0);

        // Motion Magic constraints for extend (slot 0).
        // CTRE units: rotations per second and rotations per second^2.
        public static double allowedClosedLoopErrorOut = 0.5;
        public static double maxAccelerationOut = 30000;
        public static double maxVelocityOut = 3000;

        // Motion Magic constraints for retract (slot 1).
        // CTRE units: rotations per second and rotations per second^2.
        public static double allowedClosedLoopErrorIn = 0.5;
        public static double maxAccelerationIn = 60000;
        public static double maxVelocityIn = 6000;

        // Encoder setpoints for telescopic arm height states.
        // TODO: Replace with climber-specific positions for LOW/MID/HIGH.
        public static final double lowPosition = 70.0;
        public static final double midPosition = 180.0;
        public static final double highPosition = 224.0;
        public static final double positionTolerance = 1.0;
    }
}
