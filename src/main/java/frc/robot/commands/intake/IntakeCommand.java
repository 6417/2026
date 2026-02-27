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
        if (intake.operatorIsControlling) {
            return;
        }
        if (intake.motorIsBlocked) {
            return;
        }

        // If the Robot is in the Alliance-Zone, stop the intake.
        if (!Utils.isRobotInNeutralZone() && intake.isIntakeOn) {
            intake.stop();
            return;
        }

        // Automatic intake in neutral zone disabled FOR NOW SIMPLE :D. 
        // if (!intake.isIntakeOn) {
        //     intake.ballsIn();
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
