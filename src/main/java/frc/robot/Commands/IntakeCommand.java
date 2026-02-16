package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private final double intakePercent;
    private IntakeSubsystem intake;

    public IntakeCommand(IntakeSubsystem intake) {
        this(intake, Constants.Intake.intakeSpeed);
    }

    public IntakeCommand(IntakeSubsystem intake, double intakePercent) {
        this.intakePercent = intakePercent;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setPercent(intakePercent);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
