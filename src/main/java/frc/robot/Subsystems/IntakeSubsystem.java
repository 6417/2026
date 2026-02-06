package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.FridolinsMotor;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final FridoSparkMax intakeMotor;

    public IntakeSubsystem() {
        intakeMotor = new FridoSparkMax(Constants.Intake.intakeMotorId);

        intakeMotor.setIdleMode(Constants.Intake.idleMode);
    }

    @Override
    public void periodic() {
        double currentAmps = intakeMotor.getOutputCurrent();
        double rpm = Math.abs(intakeMotor.getEncoderVelocity());
        if (currentAmps > Constants.Intake.intakeStallCurrentAmps && rpm < Constants.Intake.intakeStallRpmThreshold) {
            intakeMotor.stopMotor();
        }
    }

    public void intake() {
        setPercent(Constants.Intake.intakeSpeed);
    }

    public void outtake() {
        setPercent(Constants.Intake.outtakeSpeed);
    }

    public void setPercent(double percent) {
        double clamped = MathUtil.clamp(percent, -1.0, 1.0);
        intakeMotor.set(clamped);
    }

    public void stop() {
        intakeMotor.stopMotor();;
    }
}
