package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class IntakeCommand extends Command {
    private final double intakePercent;

    public IntakeCommand() {
        this(Constants.Intake.intakeSpeed);
    }

    public IntakeCommand(double intakePercent) {
        this.intakePercent = intakePercent;
        addRequirements(RobotContainer.intake);
    }

    @Override
    public void initialize() {
        RobotContainer.intake.setPercent(intakePercent);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
