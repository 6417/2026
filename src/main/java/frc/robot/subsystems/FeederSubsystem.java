package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.FridolinsMotor;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {
    private final FridolinsMotor singulatorMotor;
    private final FridolinsMotor feederMotor;

    public FeederSubsystem() {
        singulatorMotor = new FridoSparkMax(Constants.Feeder.singulatorMotorId);
        feederMotor = new FridoSparkMax(Constants.Feeder.feederMotorId);

        //TODO: Check if Motors are inverted

        singulatorMotor.setIdleMode(Constants.Feeder.idleMode);
        feederMotor.setIdleMode(Constants.Feeder.idleMode);
    }

    public void setPercent(double singulator, double feeder) {
        singulatorMotor.set(MathUtil.clamp(singulator, -1.0, 1.0));
        feederMotor.set(MathUtil.clamp(feeder, -1.0, 1.0));
    }

    public void setSingulatorPercent(double singulator) {
        singulatorMotor.set(MathUtil.clamp(singulator, -1.0, 1.0));
    }

    public void setFeederPercent(double feeder) {
        feederMotor.set(MathUtil.clamp(feeder, -1.0, 1.0));
    }

    public void reverse() {
        setPercent(Constants.Feeder.outtakeSingulatorSpeed, Constants.Feeder.outtakeFeederSpeed);
    }

    public void stop() {
        singulatorMotor.stopMotor();
        feederMotor.stopMotor();
    }

    public void stopFeeder() {
        feederMotor.stopMotor();
    }
    public void stopSingulator() {
        singulatorMotor.stopMotor();
    }
}
