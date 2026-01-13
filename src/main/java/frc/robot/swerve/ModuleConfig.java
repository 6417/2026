package frc.robot.swerve;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Translation2d;
import frc.fridowpi.motors.FridoFalcon500v6;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.FridolinsMotor;
import frc.fridowpi.motors.FridolinsMotor.FridoFeedBackDevice;
import frc.fridowpi.motors.FridolinsMotor.IdleMode;
import frc.fridowpi.motors.utils.FeedForwardValues;
import frc.fridowpi.motors.utils.PidValues;
import frc.fridowpi.sensors.AnalogEncoder;

public class ModuleConfig implements Cloneable {
    public String name;
    public double maxSpeed;
    public double wheelCircumference;
    public Translation2d moduleOffset;
    public double absEncoderOffset;

    public double driveGearboxRatio;
    public int driveMotorID;
    public double driveMotorStallCurrentLimit;
    public double driveMotorFreeCurrentLimit;
    public boolean driveMotorInverted;
    public PidValues drivePidValues;
    public FeedForwardValues driveFFValues;

    public int angleMotorID;
    public double angleGearboxRatio;
    public int angleMotorStallCurrentLimit;
    public int angleMotorFreeCurrentLimit;
    public double angleMotorIzone;
    public boolean angleMotorInverted;
    public PidValues anglePidValues;

    public int encoderChannel;
    public double encoderPositionOffset;

    public double encoderThicksToRotationFalcon;
    public double encoderVelocityToRPSFalcon;
    public double encoderThicksToRotationNEO;
    public double encoderVelocityToRPSNEO;

    public ModuleConfig() {
        name = null;
        maxSpeed = 0.0;
        wheelCircumference = 0.0;
        moduleOffset = null;
        absEncoderOffset = 0.0;

        driveGearboxRatio = 0.0;
        driveMotorID = -1; // Invalid ID
        driveMotorStallCurrentLimit = 0;
        driveMotorFreeCurrentLimit = 0;
        driveMotorInverted = false;
        drivePidValues = null; // Consider creating an "invalid" PidValues object if null isn't suitable
        driveFFValues = null;

        angleMotorID = -1; // Invalid ID
        angleGearboxRatio = 0.0;
        angleMotorStallCurrentLimit = 0;
        angleMotorFreeCurrentLimit = 0;
        angleMotorIzone = 0.0;
        angleMotorInverted = true;
        anglePidValues = null;

        encoderChannel = -1; // Invalid channel
        encoderPositionOffset = 0.0;

        encoderThicksToRotationFalcon = 0.0;
        encoderVelocityToRPSFalcon = 0.0;
        encoderThicksToRotationNEO = 0.0;
        encoderVelocityToRPSNEO = 0.0;
    }

    public FridoFalcon500v6 makeDriveMotor() {
        FridoFalcon500v6 driveMotor = new FridoFalcon500v6(driveMotorID);
        driveMotor.factoryDefault();
        driveMotor.asTalonFX().getConfigurator().apply(new Slot0Configs().withKP(drivePidValues.kP)
                .withKS(driveFFValues.kS).withKV(driveFFValues.kV).withKA(driveFFValues.kA));
        driveMotor.asTalonFX().getConfigurator()
                .apply(new CurrentLimitsConfigs().withStatorCurrentLimit(driveMotorFreeCurrentLimit)
                        .withSupplyCurrentLimit(driveMotorStallCurrentLimit));
        driveMotor.configEncoder(FridoFeedBackDevice.kBuildin, (int) encoderThicksToRotationFalcon);
        driveMotor.setInverted(driveMotorInverted);
        driveMotor.setPID(drivePidValues, driveFFValues);
        driveMotor.setIdleMode(IdleMode.kBrake);
        return driveMotor;
    }

    public FridolinsMotor makeAngleMotor() {
        FridoSparkMax angleMotor = new FridoSparkMax(angleMotorID);
        // angleMotor.factoryDefault();
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(angleMotorStallCurrentLimit, angleMotorFreeCurrentLimit);
        angleMotor.asSparkMax().configure(config, SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
        angleMotor.setInverted(angleMotorInverted);
        config.closedLoop.iZone(angleMotorIzone);
        angleMotor.setPID(anglePidValues);
        return angleMotor;
    }

    public AnalogEncoder makeAbsoluteEncoder() {
        AnalogEncoder encoder = new AnalogEncoder(encoderChannel);
        encoder.setPositionOffset(encoderPositionOffset);
        return encoder;
    }

    @Override
    public ModuleConfig clone() {
        try {
            ModuleConfig cloned = (ModuleConfig) super.clone();

            // Deep copy of mutable objects
            cloned.moduleOffset = moduleOffset != null ? new Translation2d(moduleOffset.getX(), moduleOffset.getY())
                    : null;
            cloned.drivePidValues = drivePidValues != null ? drivePidValues.clone() : null;
            cloned.driveFFValues = driveFFValues != null ? driveFFValues.clone() : null;
            cloned.anglePidValues = anglePidValues != null ? anglePidValues.clone() : null;

            return cloned;
        } catch (CloneNotSupportedException e) {
            throw new AssertionError("Cloning not supported", e);
        }
    }
}
