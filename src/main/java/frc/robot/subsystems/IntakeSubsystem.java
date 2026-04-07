package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.utils.PidValues;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeMotor;
    double activeRpmSetpoint;

    public IntakeSubsystem() {
        intakeMotor = new TalonFX(Constants.Intake.intakeMotorId);

        intakeMotor.setNeutralMode(Constants.Intake.idleMode);

        activeRpmSetpoint = 0; 

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
        // Could be added back for auto-intake
        // setDefaultCommand(new IntakeCommand(this));
    }
    
    @Override
    public void periodic() {
        double currentAmps = intakeMotor.getSupplyCurrent().getValueAsDouble();
        double rpms = intakeMotor.getRotorVelocity().getValueAsDouble();
        Logger.recordOutput("Intake/Current", currentAmps);
        Logger.recordOutput("Intake/RPM_Motor", rpms);
        Logger.recordOutput("Intake/RPMSetpoint", activeRpmSetpoint);
    }
    
    public void ballsIn() {
        VelocityVoltage request = new VelocityVoltage(Constants.Intake.intakeSpeedRPM);
        activeRpmSetpoint = Constants.Intake.intakeSpeedRPM;
        intakeMotor.setControl(request);
    }

    public void ballsOut() {
        VelocityVoltage request = new VelocityVoltage(Constants.Intake.outtakeSpeedRPM);
        activeRpmSetpoint = Constants.Intake.outtakeSpeedRPM;
        intakeMotor.setControl(request);
    }

    public void setPercent(double percent) {
        double clamped = MathUtil.clamp(percent, -1.0, 1.0);
        intakeMotor.set(clamped);
    }

    public void stop() {
        intakeMotor.stopMotor();
    }

    public double getCurrentOutput() {
        return intakeMotor.getSupplyCurrent().getValueAsDouble();    
    }
}
