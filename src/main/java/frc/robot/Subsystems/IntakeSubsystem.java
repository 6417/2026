package frc.robot.Subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.FridolinsMotor;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final FridolinsMotor intakeMotor;
    private final FridolinsMotor singulatorMotor;

    public IntakeSubsystem() {
        intakeMotor = new FridoSparkMax(Constants.Intake.intakeMotorId);
        singulatorMotor = new FridoSparkMax(Constants.Intake.singulatorMotorId);

        intakeMotor.setInverted(Constants.Intake.intakeMotorInverted);
        singulatorMotor.setInverted(Constants.Intake.singulatorMotorInverted);

        intakeMotor.setIdleMode(Constants.Intake.idleMode);
        singulatorMotor.setIdleMode(Constants.Intake.idleMode);
    }

    public void setPercent(double percent) {
        double clamped = MathUtil.clamp(percent, -1.0, 1.0);
        intakeMotor.set(clamped);
        singulatorMotor.set(clamped);
    }

    public void setPercent(double intake, double singulator) {
        intakeMotor.set(MathUtil.clamp(intake, -1.0, 1.0));
        singulatorMotor.set(MathUtil.clamp(singulator, -1.0, 1.0));
    }

    public void setIntakePercent(double intake) {
        intakeMotor.set(MathUtil.clamp(intake, -1.0, 1.0));
    }

    public void setSingulatorPercent(double singulator) {
        singulatorMotor.set(MathUtil.clamp(singulator, -1.0, 1.0));
    }

    public void intake() {
        setPercent(Constants.Intake.intakeSpeed, Constants.Intake.singulatorSpeed);
    }

    public void intakeOnly() {
        setIntakePercent(Constants.Intake.intakeSpeed);
    }

    public void singulateOnly() {
        setSingulatorPercent(Constants.Intake.singulatorSpeed);
    }

    public void outtake() {
        setPercent(Constants.Intake.outtakeSpeed, Constants.Intake.outtakeSingulatorSpeed);
    }

    public void stop() {
        setPercent(0.0);
    }

    public void stopIntake() {
        setIntakePercent(0.0);
    }

    public void stopSingulator() {
        setSingulatorPercent(0.0);
    }

    public boolean isBallDetected() {
        return false;
    }
}
