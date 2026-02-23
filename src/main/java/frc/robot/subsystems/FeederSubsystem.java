package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {
    private final FridoSparkMax feederMotor;

    public FeederSubsystem() {
        feederMotor = new FridoSparkMax(Constants.Feeder.motorId);

        // TODO: Check if motor is inverted.
        feederMotor.setInverted(Constants.Feeder.motorInverted);

        feederMotor.setIdleMode(Constants.Feeder.idleMode);
    }

    @Override
    public void periodic() {
    }

    public void setPercent(double feeder, double indexer) {
        feederMotor.set(MathUtil.clamp(feeder, -1.0, 1.0));
    }

    public void stop() {
        feederMotor.stopMotor();
    }
}