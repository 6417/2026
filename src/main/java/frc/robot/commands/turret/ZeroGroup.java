package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;

public class ZeroGroup extends SequentialCommandGroup {
    public ZeroGroup() {
        super.addCommands(
            new TurretZeroCommand(RobotContainer.turret),
            new WaitCommand(.5)
        );
    }
}