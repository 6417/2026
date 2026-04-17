package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeMotor;
    double activeRpsSetpoint;

    public IntakeSubsystem() {
        intakeMotor = new TalonFX(Constants.Intake.intakeMotorId);

        intakeMotor.setNeutralMode(Constants.Intake.idleMode);

        activeRpsSetpoint = 0;

        MotorOutputConfigs outputConfig = new MotorOutputConfigs();
        Slot0Configs pidSlotConfig = new Slot0Configs();
        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();

        PidValues pidValues = Constants.Intake.pid;

        outputConfig.Inverted = Constants.Intake.intakeMotorInverted;

        pidSlotConfig.kP = pidValues.kP;
        pidSlotConfig.kI = pidValues.kI;
        pidSlotConfig.kD = pidValues.kD;

        pidSlotConfig.kS = Constants.Intake.ff.kS;
        pidSlotConfig.kV = Constants.Intake.ff.kV;
        pidSlotConfig.kA = Constants.Intake.ff.kA;

        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentLimit = Constants.Intake.stallAmps;
        currentLimitsConfigs.SupplyCurrentLowerLimit = Constants.Intake.freeAmps;
        currentLimitsConfigs.SupplyCurrentLowerTime = 0.1;

        intakeMotor.getConfigurator().apply(outputConfig);
        intakeMotor.getConfigurator().apply(currentLimitsConfigs);
        intakeMotor.getConfigurator().apply(pidSlotConfig);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Intake/Current", intakeMotor.getSupplyCurrent().getValueAsDouble(), Units.Amps);
        Logger.recordOutput("Intake/RPS_Motor", intakeMotor.getRotorVelocity().getValueAsDouble(), Units.RotationsPerSecond);
        Logger.recordOutput("Intake/RPSSetpoint", activeRpsSetpoint, Units.RotationsPerSecond);
    }

    public void ballsIn() {
        VelocityVoltage request;
        if (RobotContainer.shooter.getTopRpm() > 300 || RobotContainer.shooter.getTopRpm() < -300) {
            request = new VelocityVoltage(Constants.Intake.intakeSpeedDuringShootingRPS);
            activeRpsSetpoint = Constants.Intake.intakeSpeedDuringShootingRPS;
        } else {
            request = new VelocityVoltage(Constants.Intake.intakeSpeedRPS);
            activeRpsSetpoint = Constants.Intake.intakeSpeedRPS;
        }
        intakeMotor.setControl(request);
    }

    public void ballsOut() {
        VelocityVoltage request = new VelocityVoltage(Constants.Intake.outtakeSpeedRPS);
        activeRpsSetpoint = Constants.Intake.outtakeSpeedRPS;
        intakeMotor.setControl(request);
    }

    public void setPercent(double percent) {
        double clamped = MathUtil.clamp(percent, -1.0, 1.0);
        intakeMotor.set(clamped);
    }

    public void stop() {
        intakeMotor.stopMotor();
        activeRpsSetpoint = 0;
    }

    public double getCurrentOutput() {
        return intakeMotor.getSupplyCurrent().getValueAsDouble();
    }
}
