package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private IntakeSubsystem intake;

    public IntakeCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (intake.operatorIsControlling) {
            return;
        }
        if (intake.motorIsBlocked) {
            return;
        }

        var isInNeutralZone = (DriverStation.getAlliance().get() == Alliance.Blue
                && RobotContainer.drive.getPose().getX() > Constants.Field.neutralZoneStartX) ||
                (DriverStation.getAlliance().get() == Alliance.Red
                        && RobotContainer.drive.getPose().getX() < Constants.Field.neutralZoneStartX);

        // If the Robot is in the Alliance-Zone, stop the intake.
        if (!isInNeutralZone && intake.isIntakeOn) {
            intake.stop();
            return;
        }

        // If the Robot is in the Neutral-Zone, turn on the intake to pick up balls.
        // If the Intake is already on, don't turn it on again to prevent stalling the
        // motor
        if (!intake.isIntakeOn) {
            intake.ballsIn();
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
