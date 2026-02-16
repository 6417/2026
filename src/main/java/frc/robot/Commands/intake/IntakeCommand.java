package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private final double intakePercent;
    private IntakeSubsystem intake;

    private boolean changeState = false;

    public IntakeCommand(IntakeSubsystem intake) {
        this(intake, Constants.Intake.intakeSpeed);
    }

    public IntakeCommand(IntakeSubsystem intake, double intakePercent) {
        this.intakePercent = intakePercent;
        addRequirements(intake);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (!intake.run)
        {
            intake.stop();
            return;
        }

        if ((DriverStation.getAlliance().get() == Alliance.Blue && RobotContainer.drive.getPose().getX() > Constants.Field.neutralZoneStartX) ||
            (DriverStation.getAlliance().get() == Alliance.Red && RobotContainer.drive.getPose().getX() < Constants.Field.neutralZoneStartX)) {

            // in neutral zone
            if (!intake.isIntakeOn)
                intake.setPercent(intakePercent);
            else 
                intake.stop();
        }
        else if (intake.isIntakeOn) {
            intake.setPercent(intakePercent);
        }
        else {
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
