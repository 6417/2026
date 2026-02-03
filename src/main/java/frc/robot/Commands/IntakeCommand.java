package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private final IntakeSubsystem intake;
    private final double intakePercent;
    private final double singulatorPercent;

    public IntakeCommand(IntakeSubsystem intake) {
        this(intake, Constants.Intake.intakeSpeed, Constants.Intake.singulatorSpeed);
    }

    public IntakeCommand(IntakeSubsystem intake, double intakePercent, double singulatorPercent) {
        this.intake = intake;
        this.intakePercent = intakePercent;
        this.singulatorPercent = singulatorPercent;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntakePercent(intakePercent);
        intake.setSingulatorPercent(singulatorPercent);
    }

    @Override
    public void execute() {
        if (intake.isBallDetected()) {
            intake.stop();
        }
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
