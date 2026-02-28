package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.robot.Constants;
import frc.robot.commands.intake.IntakeCommand;

public class IntakeSubsystem extends SubsystemBase {
    private final FridoSparkMax intakeMotor;

    /** Whether the intake is currently on.
     */
    public boolean isIntakeOn = false;

    /** Whether the intake motor is currently blocked e.g. by a ball.
     */
    public boolean motorIsBlocked = false;

    /** Whether the operator is currently controlling the intake.
     */
    public boolean operatorIsControlling = false;

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
        if (currentAmps > Constants.Intake.intakeStallCurrentAmps && rpms < Constants.Intake.intakeStallRpmThreshold) {
            motorIsBlocked = true;
            operatorIsControlling = true;
            stop();
        }
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
