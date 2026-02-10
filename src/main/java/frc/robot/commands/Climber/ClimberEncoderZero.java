package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ClimberEncoderZero extends Command {
    private final Timer timer = new Timer();
    private boolean isFinished = false;

    public ClimberEncoderZero() {
        // Exclusive control of the climber during zeroing.
        addRequirements(RobotContainer.climber);
    }

    @Override
    public void initialize() {
        // Drive slowly downwards until current spike indicates hard stop.
        timer.reset();
        timer.start();
        isFinished = false;
        RobotContainer.climber.setManualPercent(Constants.Climber.zeroingSpeed);
    }

    @Override
    public void execute() {
        // Finish when timeout passed and current suggests the arm hit the stop.
        if (timer.get() > Constants.Climber.zeroingTimeoutSec
                && RobotContainer.climber.getAmperage() > Constants.Climber.zeroingCurrentThreshold) {
            isFinished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop motor and zero encoder after the hard stop is reached.
        RobotContainer.climber.stop();
        RobotContainer.climber.resetEncoder();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
