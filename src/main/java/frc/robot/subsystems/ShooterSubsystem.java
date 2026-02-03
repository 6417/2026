package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.FridolinsMotor;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private final FridolinsMotor topMotor;
    private final FridolinsMotor bottomMotor;

    private double lastTopPercent = 0.0;
    private double lastBottomPercent = 0.0;

    public ShooterSubsystem() {
        topMotor = new FridoSparkMax(Constants.Shooter.topMotorId);
        bottomMotor = new FridoSparkMax(Constants.Shooter.bottomMotorId);

        topMotor.setInverted(Constants.Shooter.topMotorInverted);
        bottomMotor.setInverted(Constants.Shooter.bottomMotorInverted);

        topMotor.setIdleMode(Constants.Shooter.idleMode);
        bottomMotor.setIdleMode(Constants.Shooter.idleMode);
    }

    public void stop() {
        setPercent(0.0, 0.0);
    }

    public void setPercent(double topPercent, double bottomPercent) {
        double top = MathUtil.clamp(topPercent, -1.0, 1.0);
        double bottom = MathUtil.clamp(bottomPercent, -1.0, 1.0);
        topMotor.set(top);
        bottomMotor.set(bottom);
        lastTopPercent = top;
        lastBottomPercent = bottom;
    }

    public void shoot(double basePercent) {
        setSpinRatio(Constants.Shooter.defaultSpinRatio, basePercent);
    }

    public void setSpinRatio(double spinRatio, double basePercent) {
        double ratio = MathUtil.clamp(spinRatio, Constants.Shooter.minSpinRatio, Constants.Shooter.maxSpinRatio);
        double base = MathUtil.clamp(basePercent, -1.0, 1.0);
        double top = base * ratio;
        double bottom = base;
        setPercentPreserveRatio(top, bottom);
    }

    public void setShotAngleDegrees(double angleDeg, double basePercent) {
        double ratio = angleToSpinRatio(angleDeg);
        setSpinRatio(ratio, basePercent);
    }

    public double getLastTopPercent() {
        return lastTopPercent;
    }

    public double getLastBottomPercent() {
        return lastBottomPercent;
    }

    private double angleToSpinRatio(double angleDeg) {
        double clamped = MathUtil.clamp(angleDeg, Constants.Shooter.minAngleDeg, Constants.Shooter.maxAngleDeg);
        double t = (clamped - Constants.Shooter.minAngleDeg)
                / (Constants.Shooter.maxAngleDeg - Constants.Shooter.minAngleDeg);
        return Constants.Shooter.minSpinRatio + t * (Constants.Shooter.maxSpinRatio - Constants.Shooter.minSpinRatio);
    }

    private void setPercentPreserveRatio(double topPercent, double bottomPercent) {
        double max = Math.max(Math.abs(topPercent), Math.abs(bottomPercent));
        if (max > 1.0) {
            topPercent /= max;
            bottomPercent /= max;
        }
        setPercent(topPercent, bottomPercent);
    }
}
