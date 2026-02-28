package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.Utils;
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
        // if (intake.operatorIsControlling) {
        //     return;
        // }
        // if (intake.motorIsBlocked) {
        //     return;
        // }

        intake.ballsIn();
        // if (intake.isIntakeOn) {
        //     intake.ballsIn();
        // } else {
        //     intake.stop();
        // }
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
