package frc.robot.commands.climber;

import java.awt.Robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class FinalClimbCommand extends Command {

    boolean robotIsClimbed = false;

    public FinalClimbCommand() {
        addRequirements(RobotContainer.climber);
    }

    @Override
    public void initialize() {
        RobotContainer.climber.setManualPercent(Constants.Climber.climbSpeed);
    }

    @Override
    public void execute() {
        if (RobotContainer.climber.isClimberAtPosition(Constants.Climber.climbedPosition)) {
            robotIsClimbed = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.climber.enableServoHatchet();
        RobotContainer.climber.stop();
    }

    @Override
    public boolean isFinished() {
        return robotIsClimbed;
    }
}
