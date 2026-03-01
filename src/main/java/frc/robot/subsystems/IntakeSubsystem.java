package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final FridoSparkMax intakeMotor;

    public IntakeSubsystem() {
        intakeMotor = new FridoSparkMax(Constants.Intake.intakeMotorId);

        intakeMotor.setIdleMode(Constants.Intake.idleMode);
        intakeMotor.setInverted(Constants.Intake.intakeMotorInverted);

        SparkMaxConfig config = new SparkMaxConfig();
        config.openLoopRampRate(Constants.Intake.openLoopRampRate);
        intakeMotor.asSparkMax().configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        // Could be added back for auto-intake
        // setDefaultCommand(new IntakeCommand(this));
    }

    @Override
    public void periodic() {
        double currentAmps = intakeMotor.getOutputCurrent();
        double rpms = intakeMotor.getEncoderVelocity();
       
    }

    public void ballsIn() {
        setPercent(Constants.Intake.intakeSpeed);
    }

    public void ballsOut() {
        setPercent(Constants.Intake.outtakeSpeed);
    }

    public void setPercent(double percent) {
        double clamped = MathUtil.clamp(percent, -1.0, 1.0);
        intakeMotor.set(clamped);
    }

    public void stop() {
        intakeMotor.stopMotor();
    }
}
