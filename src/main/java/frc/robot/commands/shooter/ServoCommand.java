package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public class ServoCommand extends SequentialCommandGroup {
    public ServoCommand() {
        super.addCommands(
            new InstantCommand(() -> RobotContainer.feeder.enableServoHatchet()),
            new WaitCommand(0.3),
            new InstantCommand(() -> RobotContainer.feeder.disableServoHatchet()),
            new WaitCommand(0.5)
        );
    }
}