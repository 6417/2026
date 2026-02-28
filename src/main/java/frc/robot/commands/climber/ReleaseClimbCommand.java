package frc.robot.commands.climber;

import java.awt.Robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ClimberSubsystem.ClimberState;

public class ReleaseClimbCommand extends SequentialCommandGroup {

    public ReleaseClimbCommand() {
        addRequirements(RobotContainer.climber);
        beforeStarting(null);
        addCommands(
            new InstantCommand(() -> RobotContainer.climber.disableServoHatchet()),
            new SetClimberStateCommand(ClimberState.HIGH),
            new RelaseChuchichaestliAndHomeRelativeEncoderCommand()
        );
    }

}
