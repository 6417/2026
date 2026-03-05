package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public class ClearHatchetForMovement extends SequentialCommandGroup {

    public ClearHatchetForMovement() {
        addRequirements(RobotContainer.climber);
        
        addCommands(
            new ConditionalCommand(
                // Wenn Hatchet engaged ist: disable und warten
                new SequentialCommandGroup(
                    new InstantCommand(() -> RobotContainer.climber.disableServoHatchet()),
                    new WaitCommand(0.33)
                ),
                // Wenn Hatchet nicht engaged ist: nichts tun
                new InstantCommand(() -> {}),
                // Bedingung
                () -> RobotContainer.climber.isHatchetEngaged
            )
        );
    }
}
