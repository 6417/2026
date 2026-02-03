package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.fridowpi.motors.FridoSparkMax;
import frc.fridowpi.motors.FridolinsMotor;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private final FridolinsMotor topMotor;
    private final FridolinsMotor bottomMotor;



    public ShooterSubsystem() {
        topMotor = new FridoSparkMax(Constants.Shooter.topMotorId);
        bottomMotor = new FridoSparkMax(Constants.Shooter.bottomMotorId);

        // TODO: Verify which motor needs inversion so both wheels feed the ball forward.

        topMotor.setIdleMode(Constants.Shooter.idleMode);
        bottomMotor.setIdleMode(Constants.Shooter.idleMode);
    }

    public void stop() {
        topMotor.stopMotor();
        bottomMotor.stopMotor();
    }

    public void setPercent(double topPercent, double bottomPercent) {
        double top = MathUtil.clamp(topPercent, -1.0, 1.0);
        double bottom = MathUtil.clamp(bottomPercent, -1.0, 1.0);
        topMotor.set(top);
        bottomMotor.set(bottom);
    }

    /**
     * Set the speeds of the shooter motors based on distance to the target.
     *
     * The final implementation should use measured data points and curve fitting
     * (or interpolation) to map distance -> (top %, bottom %).
     */
    public void shootFromDistance(double distanceMeters) {
        double[] setpoint = calculateSetpoint(distanceMeters);
        setPercent(setpoint[0], setpoint[1]);
    }


    /**
     * Calculates the shooter setpoint for a given distance.
     *
     * TODO:
     * For each distance, tune top/bottom percent outputs for best trajectory.
     * 
     * RETURN:
     * index 0: topspeed
     * index 1: bottomspeed
     *
     * This method currently returns a safe default.
     */
    private double[] calculateSetpoint(double distanceMeters) {
        double[] res = {0,0};
        return res;
    }
}
