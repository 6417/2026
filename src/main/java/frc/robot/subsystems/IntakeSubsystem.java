package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.FridolinsMotor;
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

        setDefaultCommand(new IntakeCommand(this));
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
        isIntakeOn = true;
    }

    public void ballsOut() {
        setPercent(Constants.Intake.outtakeSpeed);
        isIntakeOn = true;
    }

    public void setPercent(double percent) {
        double clamped = MathUtil.clamp(percent, -1.0, 1.0);
        intakeMotor.set(clamped);
    }

    public void stop() {
        intakeMotor.stopMotor();
        isIntakeOn = false;
    }
}
