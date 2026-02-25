package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/**
 * Drives the climber slowly into the mechanical zero stop.
 *
 * Zeroing criterion:
 * - motor current rises above a configured threshold (stall/contact)
 * - then encoder position is set to configured zero.
 *
 * Safety:
 * - command also exits on timeout to avoid running forever if threshold is not reached.
 */
public class ClimberZeroCommand extends Command {
    private final Timer timer = new Timer();
    private boolean zeroDetected = false;

    public ClimberZeroCommand() {
        addRequirements(RobotContainer.climber);
    }

    @Override
    public void initialize() {
        zeroDetected = false;
        timer.restart();
    }

    @Override
    public void execute() {
        RobotContainer.climber.setManualPercent(Constants.Climber.zeroingSpeed);

        if (RobotContainer.climber.getAmperage() >= Constants.Climber.zeroingCurrentThreshold) {
            zeroDetected = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.climber.stop();
        timer.stop();

        if (!interrupted && zeroDetected) {
            RobotContainer.climber.resetEncoder();
        }
    }

    @Override
    public boolean isFinished() {
        return zeroDetected || timer.hasElapsed(Constants.Climber.zeroingTimeoutSec);
    }
}
