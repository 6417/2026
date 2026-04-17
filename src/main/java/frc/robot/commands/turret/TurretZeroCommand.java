package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/**
 * Zeroes turret rotation by gently driving to a hard stop and watching current.
 * When current rises above threshold, encoder is reset to configured zero angle.
 */
public class TurretZeroCommand extends Command {
    private boolean zeroDetected = false;
    private Timer timer;

    public TurretZeroCommand() {
        timer = new Timer();
        addRequirements(RobotContainer.turret);
    }

    @Override
    public void initialize() {
        zeroDetected = false;
        timer.reset();
        timer.start();
        // Disable soft limits during zeroing, otherwise the motor might stop before hard-stop.
        RobotContainer.turret.setVoltage(Constants.TurretSubsystem.zeroingVoltage);
        System.out.println("TurretZeroStarted");
    }

    @Override
    public void execute() {
        // Verify sign on robot: this should drive toward the mechanical end-stop used as zero reference.
        if (RobotContainer.turret.isZeroDetectedByCurrent() && timer.get() >= Constants.TurretSubsystem.zeroingTimeoutSec) {
            Timer.delay(0.2);
            zeroDetected = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.turret.stopRotationMotor();
        timer.stop();

        if (zeroDetected) {
            RobotContainer.turret.resetRotationEncoder();
        }
    }

    @Override
    public boolean isFinished() {
        return zeroDetected;
    }
}
