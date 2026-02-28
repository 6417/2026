package frc.robot.commands.climber;

import java.awt.Robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberSubsystem.ClimberState;

public class FinalClimbCommand extends Command {

    public FinalClimbCommand() {
        addRequirements(RobotContainer.climber);
    }

    @Override
    public void initialize() {
        RobotContainer.climber.setManualPercent(Constants.Climber.finalClimbSpeed);
    }

    @Override
    public void execute() {
        if (RobotContainer.climber.isMotorBlockedDetectionByVelocity(Constants.Climber.finalClimbVelocityThreshold)) {
            end(false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.climber.enableServoHatchet();
        RobotContainer.climber.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
