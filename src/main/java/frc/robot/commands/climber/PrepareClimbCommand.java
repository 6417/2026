package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class PrepareClimbCommand extends SequentialCommandGroup {

    public PrepareClimbCommand() {
        addRequirements(RobotContainer.climber);
        addCommands(
                new ClearHatchetForMovement(),
                new InstantCommand(() -> RobotContainer.climber.setPositionTop()));
    }

}
