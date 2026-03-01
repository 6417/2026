package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class PulseFeederCommand extends SequentialCommandGroup {
    public PulseFeederCommand() {
        super.addCommands(
            new InstantCommand(() -> RobotContainer.feeder.run(Constants.Feeder.defaultRPM), RobotContainer.feeder),
            new WaitCommand(Constants.Feeder.pulseForwardDuration)
            /*new InstantCommand(() -> RobotContainer.feeder.run(-Constants.Feeder.defaultRPM*0.25)),
            new WaitCommand(Constants.Feeder.pulseReverseDuration)*/
        );
    }
}