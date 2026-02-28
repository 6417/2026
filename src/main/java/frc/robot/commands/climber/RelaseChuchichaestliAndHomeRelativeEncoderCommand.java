package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class RelaseChuchichaestliAndHomeRelativeEncoderCommand extends Command {

    public RelaseChuchichaestliAndHomeRelativeEncoderCommand() {
        addRequirements(RobotContainer.climber);
    }

    @Override
    public void initialize(){
        RobotContainer.climber.startHoming();
    }

    @Override
    public void execute() {
        if (RobotContainer.climber.isMotorBlockedDetectionByAmperage(Constants.Climber.homingAmpsThreshold)) {
            RobotContainer.climber.endHoming();
            end(false);
        }

    }

    @Override
    public void end(boolean interrupted) {
        // Stop motor on release or interruption.
        RobotContainer.climber.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
