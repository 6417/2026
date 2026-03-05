package frc.robot.commands.climber;

import java.awt.Robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class PrepareClimbCommand extends SequentialCommandGroup {

    public PrepareClimbCommand() {
        addRequirements(RobotContainer.climber);
        addCommands(
                new InstantCommand(() -> RobotContainer.climber.disableServoHatchet()),
                new InstantCommand(() -> RobotContainer.climber.setPositionTop()));
    }

}
